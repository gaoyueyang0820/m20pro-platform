const {test}=require('node:test');const assert=require('node:assert/strict');
const {mapPosePresentation:p}=require('../platform_web/pose_display.js');
test('invalid localization never falls back to a plausible raw marker',()=>{
 const raw={PosX:3,PosY:4,Yaw:0,Location:1};
 assert.equal(p({mode:'live',pose:raw,pose_view:{display_pose:null,health:{display_usable:false,message:'定位未就绪'}}}).pose,null);
 assert.equal(p({mode:'live',pose:raw}).pose,null);
});
test('display smoothing does not replace the raw control pose',()=>{
 const raw={PosX:1.03,PosY:2,Yaw:0,Location:0},display={PosX:1,PosY:2,Yaw:0,Location:0};
 const state={mode:'live',pose:raw,pose_view:{display_pose:display,health:{display_usable:true,status:'stationary'}}};
 assert.equal(p(state).pose,display);assert.equal(state.pose,raw);assert.equal(state.pose.PosX,1.03);
});
test('large drift warning remains visible and demo coordinates are labelled by the server mode',()=>{
 assert.equal(p({mode:'live',pose_view:{display_pose:{PosX:2},health:{display_usable:true,severity:'warning',message:'定位跳变'}}}).warning,true);
 assert.ok(p({mode:'demo',pose:{PosX:0,PosY:0,Yaw:0}}).pose);
});
