"""Versioned map extension: register a new mapping session into an existing map.

The vendor owns per-session SLAM and loop optimization. This module aligns the
new session, fuses its cloud/grid, retains raw sessions and publishes a new map.
It never edits either input map or invokes the hidden vendor continuation flag.
"""
import io,json,math,os,re,shutil,struct,subprocess,tempfile,time
from pathlib import Path


class FusionDecisionError(ValueError):
    """A rejected publication decision, distinct from loss of saved scans."""
    def __init__(self, report):
        self.report=report
        decision=report.get('decision',{})
        self.evidence_deferred=decision.get('status') in ['unconfirmed_environment_change','single_view_conflict']
        prefix='环境变化证据不足，未生成融合候选：' if self.evidence_deferred else '新旧墙体结构尚未对齐，未发布候选：'
        super().__init__(prefix+decision.get('reason','请核对配准结果'))


def capture_continuation(base,session,poses,grid_id=0,temporal_evidence=None):
    """Describe retained data and the supported NEXT capture, not native append.

    Gaps describe this saved batch. A new capture must satisfy its own evidence
    requirements; there is no implicit cross-session trajectory optimization.
    """
    import numpy as np
    from m20.map_change_policy import ChangePolicy
    policy=ChangePolicy();poses=np.asarray(poses,dtype=float);stamps=poses[:,0]
    recent=poses[stamps>=stamps[-1]-policy.window_s]
    count=len(recent);epochs=len(set(np.floor(recent[:,0]/policy.epoch_s).astype(int)))
    span=float(recent[-1,0]-recent[0,0]) if count else 0.
    requirements=dict(min_saved_keyframes=max(3,policy.add_epochs),min_distinct_epochs=policy.add_epochs,
                      epoch_s=policy.epoch_s,min_observation_span_s=policy.add_span_s,
                      min_observer_positions=policy.add_views,min_view_separation_m=policy.view_separation_m,
                      min_trajectory_span_m=.5,required_confirmed_conflict_fraction=.8,
                      applies_to='the_same_observed_change_not_just_total_frame_count')
    coverage=dict(saved_keyframes=len(poses),recent_keyframes=count,distinct_epochs=epochs,
                  observation_span_s=round(span,3),trajectory_span_m=round(float(np.linalg.norm(np.ptp(recent[:,1:3],axis=0))),3),
                  minimum_additional_keyframes=max(0,requirements['min_saved_keyframes']-count),
                  minimum_additional_epochs=max(0,policy.add_epochs-epochs),
                  minimum_additional_span_s=round(max(0.,policy.add_span_s-span),3))
    return dict(observation_session=dict(name=Path(session).name,path=str(session),
                    native_session='session_0',poses_path=str(Path(session)/'.sessions/session_0/poses.txt'),
                    keyframes_path=str(Path(session)/'.sessions/session_0/lidar_cloud'),**coverage),
                requirements=requirements,
                next_capture=dict(method='new_extension_capture',api='/api/maps/extend-here',
                    request=dict(base=Path(base).name,grid_id=grid_id),retained_session=Path(session).name,
                    append_supported=False,automatic_cross_session_fusion=False,requires_localization=True,
                    message='原始扫描已保留；从活动旧图重新补扫，新一轮需独立满足时间和观察位置要求'),
                temporal_evidence=temporal_evidence or {})


def read_pcd(path):
    import numpy as np
    from m20.map_formats import lzf_decode
    with Path(path).open('rb') as f:
        h={}
        for _ in range(80):
            line=f.readline().decode('ascii').strip()
            if not line or line.startswith('#'):continue
            k,*v=line.split();h[k.upper()]=v
            if k.upper()=='DATA':break
        names=h['FIELDS'];sizes=list(map(int,h['SIZE']));types=h['TYPE'];counts=list(map(int,h.get('COUNT',['1']*len(names))))
        total=int(h.get('POINTS',[str(int(h['WIDTH'][0])*int(h.get('HEIGHT',['1'])[0]))])[0]);stride=sum(s*c for s,c in zip(sizes,counts))
        if not 0<total<=20_000_000 or stride*total>512*1024**2:raise ValueError('地图点云数据量超出处理范围')
        fields={};encoding=h['DATA'][0]
        if encoding=='ascii':
            raw=np.loadtxt(io.BytesIO(f.read()),dtype=np.float32,ndmin=2);offset=0
            if len(raw)!=total:raise ValueError('点云行数不完整')
            for name,count in zip(names,counts):fields[name]=raw[:,offset];offset+=count
        else:
            body=f.read()
            if encoding=='binary_compressed':
                compressed,expanded=struct.unpack('<II',body[:8]);body=lzf_decode(body[8:8+compressed],expanded)
            elif encoding!='binary':raise ValueError('不支持此点云编码')
            if len(body)!=total*stride:raise ValueError('点云长度不匹配')
            offset=0
            for name,size,typ,count in zip(names,sizes,types,counts):
                dtype=np.dtype('<'+{'F':'f','I':'i','U':'u'}[typ]+str(size))
                if encoding=='binary':fields[name]=np.ndarray((total,),dtype=dtype,buffer=body,offset=offset,strides=(stride,))
                else:fields[name]=np.ndarray((total,),dtype=dtype,buffer=body,offset=offset*total,strides=(size*count,))
                offset+=size*count
        points=np.column_stack([fields[k] for k in ['x','y','z']]+[fields.get('intensity',np.zeros(total))]).astype(np.float32)
        points=points[np.isfinite(points).all(axis=1)]
        if not len(points):raise ValueError('地图没有有效点')
        return points


def write_pcd(path,points):
    import numpy as np
    p=np.asarray(points,dtype='<f4')
    if p.shape[1]==3:p=np.column_stack([p,np.zeros(len(p),dtype=np.float32)])
    header=('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'%(len(p),len(p))).encode()
    with Path(path).open('wb') as f:f.write(header);f.write(p.tobytes())


def pose_matrix(values):
    import numpy as np
    x,y,z,qx,qy,qz,qw=map(float,values);q=np.array([qx,qy,qz,qw]);q/=np.linalg.norm(q);qx,qy,qz,qw=q
    T=np.eye(4);T[:3,3]=[x,y,z]
    T[:3,:3]=[[1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],
               [2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)],
               [2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]]
    return T


def transform(points,T):
    result=points.copy();result[:,:3]=points[:,:3]@T[:3,:3].T+T[:3,3];return result


def voxel(points,size=.15,limit=None):
    import numpy as np
    # NOS NumPy 1.17 cannot reshape an empty axis-unique input.
    if not len(points):return points
    _,index=np.unique(np.floor(points[:,:3]/size).astype(np.int32),axis=0,return_index=True)
    result=points[index]
    if limit and len(result)>limit:result=result[::math.ceil(len(result)/limit)]
    return result


def nearest_distances(points,reference):
    import numpy as np
    distances=[];reference=reference[:,:3].astype(np.float32)
    for start in range(0,len(points),96):
        p=points[start:start+96,:3].astype(np.float32)
        difference=p[:,None,:]-reference[None,:,:]
        d=np.einsum('ijk,ijk->ij',difference,difference)
        distances.extend(np.sqrt(d.min(axis=1)).tolist())
    return np.array(distances)


def validate_alignment(source,target,delta):
    import numpy as np
    delta=np.asarray(delta,dtype=float)
    if delta.shape!=(4,4) or not np.isfinite(delta).all() or not np.allclose(delta[3],[0,0,0,1],atol=1e-4):raise ValueError('配准返回无效变换')
    R=delta[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=.01) or np.linalg.det(R)<.99:raise ValueError('配准矩阵不是刚体变换')
    yaw=abs(math.atan2(R[1,0],R[0,0]));tilt=math.acos(max(-1,min(1,R[2,2])))
    if np.linalg.norm(delta[:3,3])>2 or yaw>math.radians(30) or tilt>math.radians(5):raise ValueError('新旧地图对齐偏移过大，请在旧地图重叠区域重新定位后补扫')
    sample=source[::max(1,len(source)//1500)];dist=nearest_distances(transform(sample,delta),target)
    good=dist<.25;overlap=float(good.mean());rms=float(np.sqrt((dist[good]**2).mean())) if good.any() else 99
    if overlap<.35 or rms>.15:raise ValueError('新旧地图重叠不足或配准误差过大：重叠 %.0f%%，误差 %.2fm；本轮扫描已保留，原图未修改'%(overlap*100,rms))
    return dict(overlap=round(overlap,3),rmse_m=round(rms,4),correction_m=round(float(np.linalg.norm(delta[:3,3])),4))


def structural_alignment(old,session,poses,T,delta,grid,confirmed_changes=None):
    """Check independent body-frame structure before publishing a fused map.

    Floor overlap is not wall alignment. Newly explored areas are excluded
    from old-map overlap scoring, but contradictions inside known space count.
    Thresholds are initial indoor settings, not tunnel acceptance criteria.
    """
    import numpy as np
    from m20.map_change_policy import gate_alignment
    reports=[];known_conflicts=known_count=confirmed_conflicts=0
    h,w=grid['image'].shape;res=grid['resolution'];ox,oy=grid['origin'][:2]
    for index in sorted(set([0,len(poses)//2,len(poses)-1])):
        body=read_pcd(Path(session)/('.sessions/session_0/lidar_cloud/%d.pcd'%index))
        body=body[(body[:,2]>-.30)&(body[:,2]<1.50)&(np.linalg.norm(body[:,:2],axis=1)>.75)&(np.linalg.norm(body[:,:2],axis=1)<6)]
        if len(body)<80:continue
        frame_T=T@pose_matrix(poses[index,1:]);points=voxel(transform(body,frame_T),.15,600)
        lo=points[:,:3].min(axis=0)-.4;hi=points[:,:3].max(axis=0)+.4
        target=old[((old[:,:3]>=lo)&(old[:,:3]<=hi)).all(axis=1)]
        reference_body=transform(target,np.linalg.inv(frame_T))
        target=voxel(target[(reference_body[:,2]>-.3)&(reference_body[:,2]<1.5)],.10,60000)
        if len(target)<80:continue
        distance=nearest_distances(points,target)
        gx=np.floor((points[:,0]-ox)/res).astype(int);gy=h-1-np.floor((points[:,1]-oy)/res).astype(int)
        inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h);labels=np.full(len(points),205,np.uint8)
        labels[inside]=grid['image'][gy[inside],gx[inside]]
        overlap_region=(labels!=205)|(distance<.25)
        if overlap_region.sum()<80:continue
        d=distance[overlap_region];good=d<.20
        retained=nearest_distances(voxel(target,.15,600),points)
        reports.append(dict(frame=int(index),points=int(len(d)),structural_overlap=float(good.mean()),retained_old_overlap=float((retained<.2).mean()),rmse_m=float(np.sqrt((d[good]**2).mean())) if good.any() else 99.))
        conflict=(labels>205)&(distance>.25)
        known_count+=int((labels!=205).sum());known_conflicts+=int(conflict.sum())
        confirmed_conflicts+=sum((confirmed_changes or {}).get((int(x),int(y)))==0 for x,y in zip(gx[conflict],gy[conflict]))
    temporal_confirmed=known_conflicts>0 and confirmed_conflicts/known_conflicts>=.8
    if temporal_confirmed:
        for report in reports:
            report['source_overlap']=report['structural_overlap'];report['structural_overlap']=max(report['structural_overlap'],report['retained_old_overlap'])
    metrics=dict(structural_overlap=min((r['structural_overlap'] for r in reports),default=0.),
                 rmse_m=max((r['rmse_m'] for r in reports),default=99.),
                 effective_frames=len(reports),known_free_conflict_ratio=known_conflicts/max(1,known_count),
                 correction_m=float(np.linalg.norm(delta[:3,3])),correction_yaw_rad=abs(math.atan2(delta[1,0],delta[0,0])),
                 temporal_change_confirmed=temporal_confirmed,confirmed_conflict_fraction=confirmed_conflicts/max(1,known_conflicts))
    decision=gate_alignment(metrics)
    report=dict(metrics,frames=reports,decision=decision)
    if not decision['allowed']:raise FusionDecisionError(report)
    return report


def register_pcl(source,target):
    import numpy as np
    binary=shutil.which('pcl_icp')
    if not binary:raise ValueError('NOS 缺少 pcl_icp，无法融合新旧扫描')
    with tempfile.TemporaryDirectory(prefix='m20-register-') as temp:
        folder=Path(temp);inputs=folder/'input';output=folder/'output';inputs.mkdir();output.mkdir()
        write_pcd(inputs/'reference.pcd',target);write_pcd(inputs/'session.pcd',source)
        run=subprocess.run([binary,'-d','0.6','-r','0.15','-i','60',str(inputs/'reference.pcd'),str(inputs/'session.pcd')],cwd=output,capture_output=True,text=True,timeout=120)
        if run.returncode or 'Registration failed' in run.stdout:raise ValueError('点云配准失败：'+(run.stderr+run.stdout)[-1000:])
        matrices=[];rows=[]
        for line in run.stdout.splitlines():
            try:row=[float(v) for v in line.split()]
            except ValueError:rows=[];continue
            if len(row)!=4:rows=[];continue
            rows.append(row)
            if len(rows)==4:matrices.append(np.array(rows));rows=[]
        if len(matrices)<2:raise ValueError('PCL 没有返回配准变换')
        return matrices[-1]


def read_grid(folder,grid_id=0):
    import numpy as np
    stem='occ_grid'+('_'+str(grid_id) if grid_id else '')
    if grid_id==0 and not (folder/(stem+'.pgm')).exists():stem='occ_grid_0'
    config=(folder/(stem+'.yaml')).read_text();res=float(re.search(r'^resolution:\s*([^\n]+)',config,re.M)[1]);origin=json.loads(re.search(r'^origin:\s*(\[[^\]]+\])',config,re.M)[1])
    if abs(origin[2])>1e-6:raise ValueError('续建当前要求栅格 origin yaw 为 0')
    b=(folder/(stem+'.pgm')).read_bytes();tokens=[];position=0
    while len(tokens)<4:
        match=re.match(rb'\s+|#[^\n]*\n',b[position:])
        if match:position+=len(match[0]);continue
        match=re.match(rb'\S+',b[position:]);tokens.append(match[0]);position+=len(match[0])
    kind,width,height,maximum=tokens;width,height,maximum=int(width),int(height),int(maximum)
    if maximum!=255:raise ValueError('仅支持 8 位栅格')
    if kind==b'P5':
        position+=2 if b[position:position+2]==b'\r\n' else 1
        raw=np.frombuffer(b[position:],dtype=np.uint8)
    elif kind==b'P2':raw=np.array([int(v) for v in re.sub(rb'#[^\n]*',b'',b[position:]).split()],dtype=np.uint8)
    else:raise ValueError('不支持此栅格编码')
    if raw.size!=width*height:raise ValueError('栅格长度不匹配')
    image=raw.reshape(height,width).copy()
    def value(key,default):
        match=re.search(r'^'+key+r':\s*([0-9.eE+-]+)',config,re.M);return float(match[1]) if match else default
    negate=value('negate',0);occupied=value('occupied_thresh',.65);free=value('free_thresh',.196)
    if negate not in [0,1] or not 0<=free<occupied<=1:raise ValueError('栅格占据参数无效')
    probability=image.astype(np.float32)/255 if negate else 1-image.astype(np.float32)/255
    image=np.where(probability>occupied,0,np.where(probability<free,254,205)).astype(np.uint8)
    return dict(image=image,resolution=res,origin=origin,stem=stem)


def fuse_grid(base,new,T,prior=None,confirmed_changes=None):
    import numpy as np
    res=base['resolution'];old=base['image'];fresh=new['image'];oh,ow=old.shape;nh,nw=fresh.shape
    ox,oy=base['origin'][:2];nx,ny=new['origin'][:2];nr=new['resolution']
    corners=np.array([[nx,ny,0],[nx+nw*nr,ny,0],[nx,ny+nh*nr,0],[nx+nw*nr,ny+nh*nr,0]])
    corners=transform(corners,T);lo=np.minimum([ox,oy],corners[:,:2].min(axis=0));hi=np.maximum([ox+ow*res,oy+oh*res],corners[:,:2].max(axis=0))
    lower=np.floor((lo-[ox,oy])/res+1e-8).astype(int);upper=np.ceil((hi-[ox,oy])/res-1e-8).astype(int);width,height=(upper-lower).tolist();origin=np.array([ox,oy])+lower*res
    if width*height>16_000_000:raise ValueError('融合栅格过大，请检查初始定位与地图坐标')
    score=np.zeros((height,width),dtype=np.float32);seen=np.zeros((height,width),dtype=bool)
    edited=np.where(old<90,3.,np.where(old>205,-2.,0.)).astype(np.float32)
    if prior is not None and prior.shape==old.shape:
        initial=prior.copy();expected=np.where(prior>=1,1,np.where(prior<=-1,-1,0));actual=np.sign(edited)
        changed=expected!=actual;initial[changed]=edited[changed]
    else:initial=edited
    left,top=-lower[0],upper[1]-oh;score[top:top+oh,left:left+ow]=initial;seen[top:top+oh,left:left+ow]=initial!=0
    yy,xx=np.indices(fresh.shape);points=np.column_stack([nx+(xx.ravel()+.5)*nr,ny+(nh-yy.ravel()-.5)*nr,np.zeros(fresh.size)])
    points=transform(points,T);gx=np.floor((points[:,0]-origin[0])/res).astype(int);gy=height-1-np.floor((points[:,1]-origin[1])/res).astype(int)
    values=fresh.ravel();valid=(gx>=0)&(gx<width)&(gy>=0)&(gy<height)&((values<90)|(values>205))
    ids=gy[valid]*width+gx[valid];evidence=np.where(values[valid]<90,2.,-1.)
    # Each new session contributes once per cell, regardless of source resolution.
    unique,inverse=np.unique(ids,return_inverse=True);vote=np.zeros(len(unique));np.maximum.at(vote,inverse,(evidence>0).astype(int));vote=np.where(vote>0,2.,-1.)
    flat=score.ravel();flat[unique]=np.clip(flat[unique]+vote,-6,6);seen.ravel()[unique]=True
    image=np.full((height,width),205,dtype=np.uint8);image[seen&(score>=1)]=0;image[seen&(score<=-1)]=254
    # One SLAM package is not independent temporal evidence of a changed wall.
    # Preserve known cells until the repeated-observation pipeline confirms it.
    patch=image[top:top+oh,left:left+ow];known=old!=205;conflicts=known&(patch!=old)
    deferred=int(conflicts.sum());patch[known]=old[known]
    for (x,y),value in (confirmed_changes or {}).items():
        if 0<=x<ow and 0<=y<oh and value in [0,254]:
            if conflicts[y,x] and value!=old[y,x]:deferred-=1
            patch[y,x]=value;score[top+y,left+x]=3 if value==0 else -2
    return dict(image=image,score=score,resolution=res,origin=[float(origin[0]),float(origin[1]),0],new_known_cells=int((seen.sum()-(initial!=0).sum())),deferred_known_conflicts=deferred,confirmed_known_changes=int((known&(patch!=old)).sum()))


def refine_slam_structure(old,session,poses,T,grid):
    """Use independent non-floor frames to refine an already close SLAM join."""
    import numpy as np
    from m20.map_registration import independently_validated_correction
    frames=[];h,w=grid['image'].shape;res=grid['resolution'];ox,oy=grid['origin'][:2]
    if len(poses)<3:return np.eye(4),dict(status='insufficient_independent_frames',applied=False)
    for index in sorted(set([0,len(poses)//2,len(poses)-1])):
        body=read_pcd(Path(session)/('.sessions/session_0/lidar_cloud/%d.pcd'%index))
        body=body[(body[:,2]>-.30)&(body[:,2]<1.5)&(np.linalg.norm(body[:,:2],axis=1)>.75)&(np.linalg.norm(body[:,:2],axis=1)<6)]
        frame_T=T@pose_matrix(poses[index,1:]);source=transform(body,frame_T)
        gx=np.floor((source[:,0]-ox)/res).astype(int);gy=h-1-np.floor((source[:,1]-oy)/res).astype(int)
        inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h);known=np.zeros(len(source),bool)
        known[inside]=grid['image'][gy[inside],gx[inside]]!=205;source=source[known]
        local=old[np.linalg.norm(old[:,:2]-frame_T[:2,3],axis=1)<6]
        reference=transform(local,np.linalg.inv(frame_T));target=local[(reference[:,2]>-.30)&(reference[:,2]<1.5)]
        frames.append((int(index),source,target))
    return independently_validated_correction(frames)


def fuse_maps(base,session,destination,anchor,grid_id=0,registration=register_pcl):
    import numpy as np
    base,session,destination=map(Path,[base,session,destination])
    if destination.exists() or base.resolve()==destination.resolve() or session.resolve()==destination.resolve():raise ValueError('融合结果必须另存为新版本')
    old=read_pcd(base/'full_cloud.pcd');new=read_pcd(session/'full_cloud.pcd')
    poses=np.loadtxt(session/'.sessions/session_0/poses.txt',ndmin=2)
    if poses.shape[1]!=8:raise ValueError('扫描轨迹格式不匹配')
    cloud_indices={int(p.stem) for p in (session/'.sessions/session_0/lidar_cloud').glob('*.pcd') if p.stem.isdigit()}
    if cloud_indices!=set(range(len(poses))):raise ValueError('本轮扫描轨迹与原始帧数不一致，已保留原图与采集数据')
    first=pose_matrix(poses[0,1:]);yaw=float(anchor['yaw']);anchor_T=pose_matrix([anchor['x'],anchor['y'],anchor.get('z',0),0,0,math.sin(yaw/2),math.cos(yaw/2)])
    guess=anchor_T@np.linalg.inv(first)
    first_cloud=read_pcd(session/'.sessions/session_0/lidar_cloud/0.pcd')
    source=voxel(transform(first_cloud,anchor_T),.15,14000)
    # An entire-floor cloud must not dilute the reference near this session.
    # Keep full local density for registration and overlap checks.
    lo=source[:,:3].min(axis=0)-2;hi=source[:,:3].max(axis=0)+2
    local=old[((old[:,:3]>=lo)&(old[:,:3]<=hi)).all(axis=1)]
    if len(local)<100:raise ValueError('初始定位附近没有足够原图结构，请重新定位')
    target=voxel(local,.12,60000)
    try:delta=registration(source,target);quality=validate_alignment(source,target,delta);T=delta@guess
    except ValueError as exc:
        report=dict(stage='registration',decision=dict(allowed=False,status='registration_failed',reason=str(exc)))
        report.update(capture_continuation(base,session,poses,grid_id))
        raise FusionDecisionError(report) from exc
    old_grid=read_grid(base,grid_id)
    correction,structural_correction=refine_slam_structure(old,session,poses,T,old_grid)
    if structural_correction['applied']:
        delta=correction@delta;T=delta@guess;quality=validate_alignment(source,target,delta)
    from m20.map_change_policy import gate_alignment
    correction_check=gate_alignment(dict(structural_overlap=1.,rmse_m=0.,effective_frames=1,known_free_conflict_ratio=0.,
        correction_m=float(np.linalg.norm(delta[:3,3])),correction_yaw_rad=abs(math.atan2(delta[1,0],delta[0,0]))))
    if not correction_check['allowed']:
        report=dict(stage='registration_correction',decision=correction_check)
        report.update(capture_continuation(base,session,poses,grid_id))
        raise FusionDecisionError(report)
    from m20.map_refinement import session_evidence,clear_confirmed_geometry
    evidence,evidence_report=session_evidence(session,poses,T,old_grid)
    confirmed={(x,y):value for x,y,value in evidence['confirmed_cells']} if evidence else {}
    try:structure=structural_alignment(old,session,poses,T,delta,old_grid,confirmed)
    except FusionDecisionError as exc:
        exc.report.update(capture_continuation(base,session,poses,grid_id,evidence_report))
        exc.report.update(transform=T.tolist(),registration_quality=quality,structural_correction=structural_correction)
        raise
    evidence_report['registration_source']='validated_slam_registration'
    original_points=len(old);removed_points=0
    if evidence:old,removed_points=clear_confirmed_geometry(old,old_grid,evidence)
    moved=transform(new,T)
    if evidence:moved,_=clear_confirmed_geometry(moved,old_grid,evidence)
    old_keys=np.ascontiguousarray(np.floor(old[:,:3]/.04).astype(np.int32))
    new_keys=np.ascontiguousarray(np.floor(moved[:,:3]/.04).astype(np.int32))
    key_type=np.dtype((np.void,old_keys.dtype.itemsize*3))
    h,w=old_grid['image'].shape;res=old_grid['resolution'];ox,oy=old_grid['origin'][:2]
    gx=np.floor((moved[:,0]-ox)/res).astype(int);gy=h-1-np.floor((moved[:,1]-oy)/res).astype(int)
    inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h);known=np.zeros(len(moved),bool)
    known[inside]=old_grid['image'][gy[inside],gx[inside]]!=205
    matches_old=np.isin(new_keys.view(key_type).ravel(),np.unique(old_keys.view(key_type).ravel()))
    matches_stable=np.zeros(len(moved),bool)
    if evidence and len(evidence['points']):
        stable_keys=np.ascontiguousarray(np.floor(evidence['points'][:,:3]/.10).astype(np.int32))
        moved_keys=np.ascontiguousarray(np.floor(moved[:,:3]/.10).astype(np.int32))
        matches_stable=np.isin(moved_keys.view(key_type).ravel(),np.unique(stable_keys.view(key_type).ravel()))
    accepted=~known|matches_old|matches_stable;deferred_points=int((~accepted).sum())
    moved=moved[accepted];new_keys=new_keys[accepted]
    touched=np.isin(old_keys.view(key_type).ravel(),np.unique(new_keys.view(key_type).ravel()))
    untouched=old[~touched]
    combined=np.concatenate([old[touched],moved]);keys=np.floor(combined[:,:3]/.04).astype(np.int32)
    if len(combined):
        _,inverse=np.unique(keys,axis=0,return_inverse=True);counts=np.bincount(inverse).astype(np.float64)
        merged=np.column_stack([np.bincount(inverse,weights=combined[:,i])/counts for i in range(4)]).astype(np.float32)
        merged=np.concatenate([untouched,merged])
    else:
        merged=untouched
    if not len(merged):raise ValueError('没有可生成地图的有效融合点，原始扫描已保留')
    new_grid=read_grid(session,0);prior_file=base/('.m20-grid-evidence-'+str(grid_id)+'.npy')
    prior=np.load(prior_file,allow_pickle=False) if prior_file.exists() else None
    grid=fuse_grid(old_grid,new_grid,T,prior,confirmed_changes=confirmed)
    with tempfile.TemporaryDirectory(prefix='.m20-fusion-',dir=destination.parent) as temp:
        stage=Path(temp)/'map'
        ignore=shutil.ignore_patterns('.blocks','blocks','.nav_maps','.m20-platform-manifest.json','full_cloud.pcd')
        shutil.copytree(base,stage,symlinks=True,ignore=ignore)
        for p in stage.rglob('*'):
            if p.is_symlink():
                original=(base/p.relative_to(stage)).resolve();relative=original.relative_to(base.resolve());p.unlink();p.symlink_to(os.path.relpath(stage/relative,p.parent))
        write_pcd(stage/'full_cloud.pcd',merged)
        stem=old_grid['stem'];image=grid['image'];h,w=image.shape
        (stage/(stem+'.pgm')).write_bytes(('P5\n%d %d\n255\n'%(w,h)).encode()+image.tobytes())
        (stage/(stem+'.yaml')).write_text('image: '+str(destination/(stem+'.pgm'))+'\nresolution: '+str(grid['resolution'])+'\norigin: '+json.dumps(grid['origin'])+'\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        np.save(stage/('.m20-grid-evidence-'+str(grid_id)+'.npy'),grid['score'],allow_pickle=False)
        for config in stage.glob('occ_grid*.yaml'):
            config.write_text(re.sub(r'^image:.*$',lambda m:'image: '+str(destination/(config.stem+'.pgm')),config.read_text(),flags=re.M))
        ids=stage/'occ_grid_id_map.toml'
        if ids.exists():ids.write_text(re.sub(r'^path\s*=.*$',lambda m:'path = '+json.dumps(str(destination)+'/'),ids.read_text(),flags=re.M))
        sessions=stage/'.sessions';sessions.mkdir(exist_ok=True);existing=[int(p.name.split('_')[-1]) for p in sessions.glob('session_*') if p.name.split('_')[-1].isdigit()];index=max(existing,default=-1)+1
        saved=sessions/('session_'+str(index));shutil.copytree(session/'.sessions/session_0',saved)
        # Native local poses and raw scans are preserved; the project stores their transform explicitly.
        (saved/'map_transform.json').write_text(json.dumps(dict(transform=T.tolist(),source=str(session.name)),indent=2))
        optimizer=session/'.optimizers/optimizer_0';loops=len((optimizer/'loops.txt').read_text().splitlines()) if (optimizer/'loops.txt').exists() else 0
        if optimizer.exists():shutil.copytree(optimizer,stage/'.optimizers'/('optimizer_'+str(index)),dirs_exist_ok=True)
        history=json.loads((base/'.m20-project.json').read_text()) if (base/'.m20-project.json').exists() else dict(root_map=base.name,revision=1,sessions=[])
        report=dict(base_map=base.name,map=destination.name,session=index,source_map=session.name,created=time.time(),quality=quality,
            transform=T.tolist(),base_points=original_points,new_points=len(new),fused_points=len(merged),preserved_unobserved_points=len(untouched),removed_confirmed_points=removed_points,includes_complete_base=True,keyframes=len(poses),loop_constraints=loops,new_known_cells=grid['new_known_cells'],grid_id=grid_id,
            structural_alignment=structure,structural_correction=structural_correction,deferred_known_conflicts=grid['deferred_known_conflicts'],confirmed_known_changes=grid['confirmed_known_changes'],deferred_known_region_points=deferred_points,known_region_change_policy='saved_keyframe_temporal_evidence',temporal_evidence=evidence_report)
        history['revision']+=1;history['sessions'].append(report);(stage/'.m20-project.json').write_text(json.dumps(history,ensure_ascii=False,indent=2))
        (stage/'.nav_maps/nav_map_0').mkdir(parents=True);(stage/'.nav_maps/active').symlink_to('nav_map_0')
        stage.rename(destination)
    return dict(ok=True,report=report,revision=history['revision'],map=destination.name)
