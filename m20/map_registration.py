"""Small, independently checked structural corrections for passive map updates.

Original NumPy implementation of robust planar point-to-line ICP. This does not
replace vendor localization/SLAM or optimize historical trajectories. Floors do
not constrain this fit; several differently oriented walls must constrain XY/yaw.
No upstream project code is copied. See data/reviews/autonomy-v3-20261101/sources-map.md.
"""
import math
import numpy as np


def thin_xy(points, size=.10, limit=1200):
    points=np.asarray(points,dtype=float)[:,:2]
    if not len(points):return points
    _,indices=np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
    result=points[indices]
    if len(result)>limit:result=result[::math.ceil(len(result)/limit)]
    return result


def nearest(points,target):
    indices=[];distances=[]
    for offset in range(0,len(points),128):
        delta=points[offset:offset+128,None,:]-target[None,:,:]
        squared=np.einsum('ijk,ijk->ij',delta,delta);chosen=squared.argmin(axis=1)
        indices.extend(chosen);distances.extend(np.sqrt(squared[np.arange(len(chosen)),chosen]))
    return np.asarray(indices,dtype=int),np.asarray(distances)


def wall_normals(target):
    """Reject corners/unstructured clusters; use local line normals as anchors."""
    buckets={};size=.35
    for i,p in enumerate(target):buckets.setdefault(tuple(np.floor(p/size).astype(int)),[]).append(i)
    normals=np.zeros_like(target);valid=np.zeros(len(target),bool)
    for i,p in enumerate(target):
        key=np.floor(p/size).astype(int)
        near=[j for dx in [-1,0,1] for dy in [-1,0,1] for j in buckets.get((key[0]+dx,key[1]+dy),[])]
        patch=target[near];patch=patch[np.linalg.norm(patch-p,axis=1)<.35]
        if len(patch)<5:continue
        values,vectors=np.linalg.eigh(np.cov(patch.T))
        if values[-1]<.008 or values[0]/max(values[-1],1e-9)>.12:continue
        normals[i]=vectors[:,0];valid[i]=True
    return normals,valid


def fit_structure(source,target,max_translation=.20,max_yaw=math.radians(5)):
    """Return correction + diagnostics; an unconstrained single wall stays put.

    A bounded local fit is intentional: newly moved objects cannot pull the whole
    map into a different room or make a large pose error look like map refinement.
    """
    source=thin_xy(source,limit=800);target=thin_xy(target,limit=1600)
    report=dict(status='insufficient_anchors',applied=False)
    identity=np.eye(4)
    if len(source)<30 or len(target)<30:return identity,report
    normals,valid=wall_normals(target)
    if valid.sum()<30:return identity,report
    center=source.mean(axis=0);moving=source.copy();rotation=np.eye(2);translation=np.zeros(2)
    before=None;condition=0
    for iteration in range(18):
        indices,distance=nearest(moving,target);use=(distance<.25)&valid[indices]
        if use.sum()<24:return identity,dict(report,status='insufficient_correspondences')
        p=moving[use];n=normals[indices[use]];q=target[indices[use]]
        residual=np.einsum('ij,ij->i',n,p-q)
        # Robust loss tolerates a minority of newly added/removed structures.
        weight=np.minimum(1.,.055/np.maximum(abs(residual),1e-8))
        relative=p-center
        J=np.column_stack([n[:,0],n[:,1],-n[:,0]*relative[:,1]+n[:,1]*relative[:,0]])
        normal=J.T@(weight[:,None]*J)
        eigen=np.linalg.eigvalsh(normal);condition=float(eigen[0]/max(eigen[-1],1e-9))
        if condition<1e-4:return identity,dict(report,status='degenerate_structure',constraint_ratio=condition)
        error=float(np.sqrt(np.mean(residual**2)))
        if before is None:before=error
        step=np.linalg.solve(normal,-J.T@(weight*residual))
        angle=float(np.clip(step[2],-.025,.025));c,s=math.cos(angle),math.sin(angle);R=np.array([[c,-s],[s,c]])
        shift=np.clip(step[:2],-.05,.05)+center-R@center
        moving=moving@R.T+shift;translation=R@translation+shift;rotation=R@rotation
        displacement=float(np.linalg.norm(rotation@center+translation-center));yaw=math.atan2(rotation[1,0],rotation[0,0])
        if displacement>max_translation or abs(yaw)>max_yaw:return identity,dict(report,status='correction_out_of_range',correction_m=displacement,yaw_rad=yaw)
        if np.linalg.norm(step[:2])<.0005 and abs(angle)<.0002:break
    indices,distance=nearest(moving,target);use=(distance<.25)&valid[indices]
    residual=np.einsum('ij,ij->i',normals[indices[use]],moving[use]-target[indices[use]])
    after=float(np.sqrt(np.mean(residual**2)))
    candidate=np.eye(4);candidate[:2,:2]=rotation;candidate[:2,3]=translation
    report=dict(status='candidate',applied=False,before_rmse_m=before,after_rmse_m=after,
                constraint_ratio=condition,correction_m=displacement,yaw_rad=yaw,iterations=iteration+1,anchors=int(use.sum()))
    if before is None or before<.018 or after>before*.75 or before-after<.008:
        return identity,dict(report,status='no_material_improvement')
    return candidate,report


def structure_error(source,target,transform=None):
    source=thin_xy(source,limit=600);target=thin_xy(target,limit=1600)
    if transform is not None:source=source@transform[:2,:2].T+transform[:2,3]
    normals,valid=wall_normals(target)
    if len(source)<30 or len(target)<30 or valid.sum()<24:return None
    indices,distance=nearest(source,target);use=(distance<.25)&valid[indices]
    if use.sum()<24:return None
    residual=np.einsum('ij,ij->i',normals[indices[use]],source[use]-target[indices[use]])
    return dict(rmse_m=float(np.sqrt(np.mean(residual**2))),matched=int(use.sum()),points=len(source))


def independently_validated_correction(frames):
    """Middle observation fits; first/last observations independently validate."""
    if len(frames)<3:return np.eye(4),dict(status='insufficient_independent_frames',applied=False)
    _,source,target=frames[1];correction,report=fit_structure(source,target)
    if report['status']!='candidate':return correction,report
    validation=[]
    for index,source,target in [frames[0],frames[-1]]:
        before=structure_error(source,target);after=structure_error(source,target,correction)
        if before is None or after is None or after['matched']<before['matched']*.9 or after['rmse_m']>before['rmse_m']*.85:
            return np.eye(4),dict(report,status='independent_validation_failed',applied=False)
        validation.append(dict(frame=index,before=before,after=after))
    report.update(status='validated_structure_correction',applied=True,transform=correction.tolist(),independent_frames=validation)
    return correction,report
