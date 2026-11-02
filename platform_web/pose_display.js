'use strict';
// Display only. Goal selection, localization initialization and control keep S.pose.
function mapPosePresentation(state){
 const view=state?.pose_view;
 if(view){
  const usable=view.health?.display_usable===true&&view.display_pose;
  return {pose:usable?view.display_pose:null,message:view.health?.message||'等待定位',warning:view.health?.severity==='warning',status:view.health?.status};
 }
 const raw=state?.pose,valid=raw&&Number.isFinite(raw.PosX)&&Number.isFinite(raw.PosY)&&Number.isFinite(raw.Yaw)&&(state.mode==='demo'||raw.Location===0);
 return {pose:valid?raw:null,message:valid?'':'定位未就绪，当前位置不可用',warning:false,status:valid?'raw':'unavailable'};
}
if(typeof module!=='undefined'&&module.exports)module.exports={mapPosePresentation};
