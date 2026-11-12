"""Passive map refinement from quality-checked localization observations.

No SLAM launch, control output or active-map switch. Repeated observations add
stable geometry; only repeatedly observed floor can clear old occupancy.
"""
import hashlib,json,math,os,re,shutil,tempfile,time
from contextlib import contextmanager
from pathlib import Path
import numpy as np
from m20.map_fusion import read_grid,read_pcd,write_pcd,pose_matrix,transform,voxel,nearest_distances
from m20.autonomy_sensing import supported_floor


def base_fingerprint(folder,grid_id=0):
    grid=read_grid(folder,grid_id);h=hashlib.sha256()
    for p in [folder/'full_cloud.pcd',folder/(grid['stem']+'.pgm'),folder/(grid['stem']+'.yaml')]:
        with p.open('rb') as f:
            for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def observation_frames(files):
    """Read optional calibrated single-sensor origins without inventing them."""
    for file in files:
        with np.load(file,allow_pickle=False) as frame:
            item=dict(points=np.asarray(frame['points']),pose=np.asarray(frame['pose']),stamp=float(frame['stamp']))
            if 'sensor_origin_world' in frame and 'sensor_origin_trusted' in frame:
                trusted=np.asarray(frame['sensor_origin_trusted'])
                item.update(sensor_origin_world=np.asarray(frame['sensor_origin_world']),
                            sensor_origin_trusted=bool(trusted.item()) if trusted.shape==() else False)
            yield item


def batch_structures(files,old,grid):
    """Fit against non-floor structure in mapped overlap, not new rooms."""
    frames=[];h,w=grid['image'].shape;ox,oy=grid['origin'][:2];res=grid['resolution']
    for index in sorted(set([0,len(files)//2,len(files)-1])):
        with np.load(files[index],allow_pickle=False) as f:points=f['points'];pose=f['pose']
        T=pose_matrix(pose);body=transform(points,np.linalg.inv(T))
        source=points[(body[:,2]>-.30)&(body[:,2]<1.5)&(np.linalg.norm(body[:,:2],axis=1)>.75)]
        gx=np.floor((source[:,0]-ox)/res).astype(int);gy=h-1-np.floor((source[:,1]-oy)/res).astype(int)
        inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h);known=np.zeros(len(source),bool)
        known[inside]=grid['image'][gy[inside],gx[inside]]!=205;source=source[known]
        local=old[np.linalg.norm(old[:,:2]-T[:2,3],axis=1)<6]
        body_old=transform(local,np.linalg.inv(T));target=local[(body_old[:,2]>-.30)&(body_old[:,2]<1.5)]
        frames.append((index,source,target))
    return frames


@contextmanager
def aligned_observations(files,old,grid):
    """Correct one small rigid localization bias, verified on withheld frames.

    The middle frame estimates a correction. First and last are never used in
    that estimate; both must improve against old structure before the correction
    is used for any map evidence. Original files/poses remain untouched.
    """
    from m20.map_registration import independently_validated_correction
    frames=batch_structures(files,old,grid)
    correction,report=independently_validated_correction(frames)
    if not report['applied']:
        yield files,report;return
    with tempfile.TemporaryDirectory(prefix='m20-aligned-observations-') as temp:
        corrected=[]
        for file in files:
            with np.load(file,allow_pickle=False) as frame:values={key:frame[key] for key in frame.files}
            values['points']=transform(values['points'],correction).astype(np.float32)
            values['pose']=matrix_pose(correction@pose_matrix(values['pose']))
            if 'sensor_origin_world' in values:
                values['sensor_origin_world']=transform(np.asarray(values['sensor_origin_world']).reshape(1,3),correction)[0]
            out=Path(temp)/file.name;np.savez(out,**values);corrected.append(out)
        yield corrected,report


def collect_evidence(files,grid):
    from m20.map_change_policy import classify_cell,ChangePolicy
    policy=ChangePolicy()
    geometry={};cells={};poses=[];accepted=0;ground_heights={}
    height,width=grid['image'].shape;res=grid['resolution'];ox,oy=grid['origin'][:2]
    # Old sightings must age out; a once-occupied cell must not become a ghost
    # wall forever. Use sensor time, not the computer/robot wall-clock offset.
    stamps=[]
    for file in files:
        with np.load(file,allow_pickle=False) as frame:stamps.append(float(frame['stamp']))
    if not stamps or not np.isfinite(stamps).all():raise ValueError('观测时间无效')
    latest=max(stamps)
    for file,source_stamp in zip(files,stamps):
        if source_stamp<latest-policy.window_s:continue
        with np.load(file,allow_pickle=False) as frame:
            pts=np.asarray(frame['points'],dtype=np.float32);pose=np.asarray(frame['pose'],dtype=float);stamp=float(frame['stamp'])
        if pts.ndim!=2 or pts.shape[1]!=3 or len(pts)>60000 or pose.shape!=(7,) or not np.isfinite(pts).all() or not np.isfinite(pose).all() or not math.isfinite(stamp):raise ValueError('采集记录格式无效')
        T=pose_matrix(pose);body=transform(pts,np.linalg.inv(T));distance=np.linalg.norm(body[:,:2],axis=1)
        pts=pts[(distance>.55)&(distance<6)];body=body[(distance>.55)&(distance<6)]
        if not len(pts):continue
        floor=supported_floor(body);free={tuple(p) for p in floor['floor']};blocked={tuple(p) for p in floor['obstacles']}
        epoch=math.floor(stamp/10);accepted+=1;poses.append(pose[:3].tolist())
        # A voxel is counted once per ten-second visit, not once per laser return.
        keys=np.floor(pts/.10).astype(np.int32);_,idx=np.unique(keys,axis=0,return_index=True)
        for key,p in zip(keys[idx],pts[idx]):
            k=tuple(key);row=geometry.setdefault(k,[np.zeros(3),0,set(),stamp,stamp,[]])
            if epoch not in row[2]:
                row[0]+=p;row[1]+=1;row[2].add(epoch);row[4]=max(row[4],stamp)
                row[5].append(dict(stamp=stamp,vote=1,origin=pose[:2].tolist()))
        xy=np.rint(body[:,:2]/.2).astype(int);gx=np.floor((pts[:,0]-ox)/res).astype(int);gy=height-1-np.floor((pts[:,1]-oy)/res).astype(int)
        votes={};floor_points={}
        for p,x,y,z in zip(xy,gx,gy,pts[:,2]):
            key=tuple(p);vote=1 if key in blocked else -1 if key in free else 0
            if vote:votes[(x,y)]=max(vote,votes.get((x,y),-1))
            if vote==-1:floor_points.setdefault((x,y),[]).append(float(z))
        for cell,vote in votes.items():
            cells.setdefault(cell,[]).append(dict(stamp=stamp,vote=vote,origin=pose[:2].tolist()))
            if vote==-1 and (cell not in ground_heights or stamp>=ground_heights[cell][0]):ground_heights[cell]=(stamp,float(np.median(floor_points[cell])))
    stable=[r[0]/r[1] for r in geometry.values() if r[1]>=3 and r[4]-r[3]>=20 and classify_cell(205,r[5],registration_ok=True,now=latest,policy=policy)['action']=='occupy']
    confirmed={};classes={}
    for (x,y),visits in cells.items():
        base=int(grid['image'][y,x]) if 0<=x<width and 0<=y<height else 205
        decision=classify_cell(base,visits,registration_ok=True,now=latest,policy=policy)
        label=decision['classification'];classes[label]=classes.get(label,0)+1
        if decision['action']=='occupy':confirmed[x,y]=0
        elif decision['action']=='clear':confirmed[x,y]=254
    # Do not re-add a previously stable object from early frames after later
    # ground observations show that it moved away. This does not delete old-map
    # geometry: clearing the original still needs the stronger cell decision.
    latest_free={cell for cell,visits in cells.items() if max(visits,key=lambda r:r['stamp'])['vote']==-1}
    def still_present(point):
        key=(math.floor((point[0]-ox)/res),height-1-math.floor((point[1]-oy)/res))
        floor=ground_heights.get(key)
        return not(key in latest_free and floor and floor[1]+.12<point[2]<floor[1]+1.5)
    stable=np.asarray([p for p in stable if still_present(p)],dtype=np.float32).reshape(-1,3)
    from m20.map_visibility import confirmed_surface_removals
    ray_removed,visibility=confirmed_surface_removals(stable,observation_frames(files),now=latest)
    stable=stable[~ray_removed]
    span=float(np.linalg.norm(np.ptp(np.array(poses)[:,:2],axis=0))) if poses else 0
    if span<.5:stable=[];confirmed={}
    minx=int(min([0]+[p[0] for p in confirmed]));maxx=int(max([width-1]+[p[0] for p in confirmed]))
    miny=int(min([0]+[p[1] for p in confirmed]));maxy=int(max([height-1]+[p[1] for p in confirmed]))
    if (maxx-minx+1)*(maxy-miny+1)>16_000_000:raise ValueError('更新范围过大，请检查定位与坐标')
    image=np.full((maxy-miny+1,maxx-minx+1),205,np.uint8);image[-miny:height-miny,-minx:width-minx]=grid['image']
    added=cleared=0
    for (x,y),value in confirmed.items():
        if value==0:added+=int(image[y-miny,x-minx]!=0)
        else:cleared+=int(image[y-miny,x-minx]!=254)
        image[y-miny,x-minx]=value
    return dict(points=np.asarray(stable,dtype=np.float32).reshape(-1,3),image=image,occupied_added=added,free_updated=cleared,
                origin=[ox+minx*res,oy+(height-1-maxy)*res,0],old_offset=[-minx,-miny],
                frames=accepted,trajectory_span_m=round(span,2),observed_cells=len(cells),change_classifications=classes,evidence_window_s=policy.window_s,latest_stamp=latest,
                registration_source='quality_gated_localization_recorder',requires_manual_activation=True,visibility_evidence=visibility,
                confirmed_cells=[[int(x),int(y),int(value)] for (x,y),value in confirmed.items()])


def matrix_pose(T):
    """XYZ + xyzw quaternion for a validated rigid transform."""
    R=T[:3,:3];trace=float(np.trace(R))
    if trace>0:
        s=math.sqrt(trace+1)*2;q=[(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s,s/4]
    else:
        i=int(np.argmax(np.diag(R)));j=(i+1)%3;k=(i+2)%3;s=math.sqrt(max(0,1+R[i,i]-R[j,j]-R[k,k]))*2
        q=[0.,0.,0.,0.];q[i]=s/4;q[j]=(R[j,i]+R[i,j])/s;q[k]=(R[k,i]+R[i,k])/s;q[3]=(R[k,j]-R[j,k])/s
    q=np.asarray(q,dtype=float);q/=np.linalg.norm(q)
    return np.r_[T[:3,3],q]


def session_evidence(session,poses,T,grid):
    """Reuse saved SLAM frames for SAME-session old-area changes.

    This bounded batch does not replace long-tunnel streaming. No keyframes are
    silently subsampled: over-budget batches defer changes and retain raw data.
    """
    stamps=poses[:,0]
    if not np.isfinite(stamps).all() or np.any(np.diff(stamps)<0):raise ValueError('SLAM关键帧时间非单调，不能作为时序变化证据')
    selected=np.flatnonzero(stamps>=stamps[-1]-120)
    if len(selected)<3:return None,dict(state='deferred',reason='fewer_than_three_keyframes',frames=int(len(selected)))
    if len(selected)>120:return None,dict(state='deferred',reason='keyframe_budget_exceeded',frames=int(len(selected)))
    total=0
    with tempfile.TemporaryDirectory(prefix='m20-session-evidence-') as temp:
        files=[]
        for index in selected:
            body=read_pcd(Path(session)/('.sessions/session_0/lidar_cloud/%d.pcd'%index))[:,:3]
            radius=np.linalg.norm(body[:,:2],axis=1);body=voxel(body[(radius>.55)&(radius<6)],.06)
            total+=len(body)
            if len(body)>60000 or total>1_500_000:return None,dict(state='deferred',reason='point_budget_exceeded',frames=int(len(selected)),points=int(total))
            # Non-level / invalid floor cannot supply clearing evidence. Defer
            # the batch rather than ignore an inconvenient contradictory frame.
            try:supported_floor(body)
            except ValueError:return None,dict(state='deferred',reason='ground_observation_unverified',frame=int(index))
            frame_T=T@pose_matrix(poses[index,1:]);file=Path(temp)/('frame-%06d.npz'%index)
            np.savez(file,points=transform(body,frame_T).astype(np.float32),pose=matrix_pose(frame_T),stamp=float(stamps[index]));files.append(file)
        evidence=collect_evidence(files,grid)
    report={k:v for k,v in evidence.items() if k not in ['points','image','old_offset','confirmed_cells']}
    report.update(state='processed',source='saved_slam_keyframes',registration_source='supplied_slam_transform_pending_structure_check',
                  points=int(total),keyframe_limit=120,point_limit=1_500_000,older_keyframes_deferred=int(len(poses)-len(selected)))
    return evidence,report


def clear_confirmed_geometry(points,grid,evidence):
    """Remove above-floor geometry only at independently confirmed clear cells."""
    h,w=grid['image'].shape;res=grid['resolution'];ox,oy=grid['origin'][:2]
    gx=np.floor((points[:,0]-ox)/res).astype(int);gy=h-1-np.floor((points[:,1]-oy)/res).astype(int)
    valid=(gx>=0)&(gx<w)&(gy>=0)&(gy<h);remove=np.zeros(len(points),bool)
    left,top=evidence['old_offset'];cleared=(grid['image']<90)&(evidence['image'][top:top+h,left:left+w]>205);stable=evidence['points']
    for x,y in zip(*np.where(cleared)[::-1]):
        floor=stable[(abs(stable[:,0]-(ox+(x+.5)*res))<res)&(abs(stable[:,1]-(oy+(h-y-.5)*res))<res)] if len(stable) else []
        if len(floor):
            z=float(np.median(np.asarray(floor)[:,2]));remove|=valid&(gx==x)&(gy==y)&(points[:,2]>z+.12)&(points[:,2]<z+1.5)
    return points[~remove],int(remove.sum())


def validate_refinement_structure(files,old,grid,temporal_change_confirmed=False):
    """Independent structure check prevents coherent localization drift from
    being confirmed merely by recording the same displaced wall many times.
    Absence of old overlap is reported, not misreported as alignment success.
    """
    from m20.map_change_policy import gate_alignment
    reports=[]
    for index in sorted(set([0,len(files)//2,len(files)-1])):
        with np.load(files[index],allow_pickle=False) as frame:
            points=np.asarray(frame['points'],dtype=np.float32);T=pose_matrix(frame['pose'])
        body=transform(points,np.linalg.inv(T));mask=(body[:,2]>-.30)&(body[:,2]<1.5)&(np.linalg.norm(body[:,:2],axis=1)>.75)
        source=voxel(points[mask],.15,500)
        h,w=grid['image'].shape;res=grid['resolution'];ox,oy=grid['origin'][:2]
        gx=np.floor((source[:,0]-ox)/res).astype(int);gy=h-1-np.floor((source[:,1]-oy)/res).astype(int)
        inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h);known=np.zeros(len(source),bool)
        known[inside]=grid['image'][gy[inside],gx[inside]]!=205;source=source[known]
        if len(source)<24:continue
        target=old[np.linalg.norm(old[:,:2]-T[:2,3],axis=1)<7]
        target_body=transform(target,np.linalg.inv(T))
        radius=np.linalg.norm(target_body[:,:2],axis=1)
        target=voxel(target[(target_body[:,2]>-.30)&(target_body[:,2]<1.5)&(radius>.75)&(radius<6)],.10,60000)
        if len(target)<24:continue
        d=nearest_distances(source,target);good=d<.2;source_overlap=float(good.mean())
        # A persistent new object can lower source->old overlap even while the
        # original walls still align. Existing visible structure provides the
        # independent anchor; a coherently displaced wall fails both directions.
        retained=nearest_distances(voxel(target,.15,500),source);retained_overlap=float((retained<.2).mean())
        overlap=max(source_overlap,retained_overlap) if temporal_change_confirmed else source_overlap
        reports.append(dict(frame=int(index),structural_overlap=overlap,source_overlap=source_overlap,retained_old_overlap=retained_overlap,
                            rmse_m=float(np.sqrt((d[good]**2).mean())) if good.any() else 99.))
    if not reports:return dict(status='no_existing_structure_overlap',effective_frames=0,source='quality_gated_localization_recorder')
    metrics=dict(structural_overlap=min(r['structural_overlap'] for r in reports),rmse_m=max(r['rmse_m'] for r in reports),
                 effective_frames=len(reports),correction_m=0.,correction_yaw_rad=0.,known_free_conflict_ratio=0.,temporal_change_confirmed=temporal_change_confirmed)
    decision=gate_alignment(metrics)
    if not decision['allowed']:raise ValueError('重复观测与旧墙体结构不一致，先恢复定位再更新：'+decision['reason'])
    return dict(metrics,decision=decision,frames=reports)



def refine_map(base,session,destination,grid_id,expected):
    base,session,destination=map(Path,[base,session,destination])
    if destination.exists() or destination.is_symlink():raise ValueError('更新版本已存在')
    if base_fingerprint(base,grid_id)!=expected:raise ValueError('原地图内容已变化，请重新开启地图优化')
    files=sorted(session.glob('frame-*.npz'))
    if not 3<=len(files)<=500:raise ValueError('有效观测不足或超过本轮容量')
    if sum(p.stat().st_size for p in files)>256*1024**2:raise ValueError('本轮观测过大')
    grid=read_grid(base,grid_id);old=read_pcd(base/'full_cloud.pcd')
    with aligned_observations(files,old,grid) as (aligned,correction):
        return _publish_refinement(base,session,destination,grid_id,expected,aligned,files,grid,old,correction)


def _publish_refinement(base,session,destination,grid_id,expected,files,original_files,grid,old,correction):
    from m20.map_formats import make_map_readable
    e=collect_evidence(files,grid)
    if len(e['points'])==0 and not(e['occupied_added'] or e['free_updated']):
        return dict(ok=True,unchanged=True,message='已保存观测，重复观测尚不足以确认地图变化',frames=len(files))
    alignment=validate_refinement_structure(files,old,grid,bool(e['occupied_added'] or e['free_updated']))
    # New geometry needs repeated independent visits. Existing points are kept;
    # a repeatedly cleared obstacle cell may remove above-floor old points only.
    pts=e['points'];retained,removed_points=clear_confirmed_geometry(old,grid,e)
    from m20.map_visibility import confirmed_surface_removals
    ray_removed,old_visibility=confirmed_surface_removals(retained,observation_frames(files),now=e['latest_stamp'])
    retained=retained[~ray_removed];removed_points+=int(ray_removed.sum())
    # Unobserved old voxels retain all their points and intensity values exactly.
    keys=np.floor(retained[:,:3]/.05).astype(np.int64);newkeys={tuple(p) for p in np.floor(pts/.05).astype(np.int64)}
    observed=np.array([tuple(p) in newkeys for p in keys],dtype=bool)
    existing=retained[observed];untouched=retained[~observed]
    combined=np.concatenate([existing,np.column_stack([pts,np.zeros(len(pts))])]).astype(np.float32)
    merged=combined;changed_geometry=0
    if len(combined):
        _,inverse=np.unique(np.floor(combined[:,:3]/.05).astype(np.int64),axis=0,return_inverse=True)
        n=np.bincount(inverse);merged=np.column_stack([np.bincount(inverse,weights=combined[:,i])/n for i in range(4)]).astype(np.float32)
        old_counts=np.bincount(inverse[:len(existing)],minlength=len(n));intensity=np.bincount(inverse[:len(existing)],weights=existing[:,3],minlength=len(n));valid_intensity=old_counts>0
        merged[valid_intensity,3]=intensity[valid_intensity]/old_counts[valid_intensity]
        old_mean=np.column_stack([np.bincount(inverse[:len(existing)],weights=existing[:,i],minlength=len(n))/np.maximum(old_counts,1) for i in range(3)])
        changed_geometry=int((old_counts==0).sum())+int((np.linalg.norm(merged[:,:3]-old_mean,axis=1)[valid_intensity]>.01).sum())
    if not changed_geometry and not removed_points and not(e['occupied_added'] or e['free_updated']):
        return dict(ok=True,unchanged=True,message='观测已保存，尚未确认需要更新的地图变化',frames=len(files))
    merged=np.concatenate([untouched,merged])
    if not len(merged):return dict(ok=True,unchanged=True,message='观测已保存，本轮没有可生成地图的有效点云')
    report={k:v for k,v in e.items() if k not in ['points','image','old_offset','confirmed_cells']}
    report.update(base_map=base.name,map=destination.name,created=time.time(),stable_points=len(pts),old_points=len(old),new_points=len(merged),preserved_unobserved_points=len(untouched),removed_points=removed_points,method='structure_aligned_temporal_visibility',grid_id=grid_id,includes_complete_base=True,structural_alignment=alignment,structural_correction=correction,old_surface_visibility=old_visibility)
    with tempfile.TemporaryDirectory(prefix='.m20-refine-',dir=destination.parent) as temp:
        stage=Path(temp)/'map';shutil.copytree(base,stage,symlinks=True,ignore=shutil.ignore_patterns('.blocks','blocks','.nav_maps','.m20-platform-manifest.json','full_cloud.pcd'))
        for p in stage.rglob('*'):
            if p.is_symlink():
                resolved=(base/p.relative_to(stage)).resolve();relative=resolved.relative_to(base.resolve());p.unlink();p.symlink_to(os.path.relpath(stage/relative,p.parent))
        write_pcd(stage/'full_cloud.pcd',merged)
        for raster in stage.glob('occ_grid*.pgm'):
            if raster.is_symlink():
                content=(base/raster.name).read_bytes();raster.unlink();raster.write_bytes(content)
        stem=grid['stem'];nh,nw=e['image'].shape;pgm=stage/(stem+'.pgm')
        if pgm.is_symlink():pgm.unlink()
        pgm.write_bytes(('P5\n%d %d\n255\n'%(nw,nh)).encode()+e['image'].tobytes())
        # Clear stale extension evidence; this version is a new occupancy baseline.
        (stage/('.m20-grid-evidence-'+str(grid_id)+'.npy')).unlink(missing_ok=True)
        for config in stage.glob('occ_grid*.yaml'):
            body=re.sub(r'^image:.*$',lambda m:'image: '+str(destination/(config.stem+'.pgm')),(base/config.name).read_text(),flags=re.M)
            if config.stem==stem:
                body=re.sub(r'^origin:.*$','origin: '+json.dumps(e['origin']),body,flags=re.M)
                for key,value in [('negate',0),('occupied_thresh',.65),('free_thresh',.196)]:body=re.sub(r'^'+key+r':.*$',key+': '+str(value),body,flags=re.M)
            if config.is_symlink():config.unlink()
            config.write_text(body)
        ids=stage/'occ_grid_id_map.toml'
        if ids.is_symlink():
            content=(base/ids.name).read_text();ids.unlink();ids.write_text(content)
        if ids.exists():ids.write_text(re.sub(r'^path\s*=.*$',lambda m:'path = '+json.dumps(str(destination)+'/'),ids.read_text(),flags=re.M))
        meta=stage/'.m20-project.json';project=json.loads(meta.read_text()) if meta.exists() else dict(root_map=base.name,revision=1,sessions=[])
        project['revision']+=1;project.setdefault('refinements',[]).append(report);meta.write_text(json.dumps(project,ensure_ascii=False,indent=2))
        (stage/'.nav_maps/nav_map_0').mkdir(parents=True);(stage/'.nav_maps/active').symlink_to('nav_map_0')
        (stage/'.m20-refinement.json').write_text(json.dumps(dict(report,source_fingerprint=expected,observations=[dict(file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in original_files]),ensure_ascii=False))
        make_map_readable(stage)
        stage.rename(destination)
    return dict(ok=True,map=destination.name,report=report,activated=False,message='完整旧地图与行走更新已合并为新版本，点击启用后生效')
