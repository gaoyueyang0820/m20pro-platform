// Product copy contracts, not implementation/diagnostic wording restrictions.
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const app=fs.readFileSync('platform_web/app.js','utf8');
function render(name,mode='live'){
 const c=vm.createContext({S:{mode,poses:[]},selectedLeg:'FL',selectedMap:()=>null,heading:()=>'',card:(name,body)=>name+body,alert:s=>s,stage:()=>'',
  btn:(label,id)=>`<button data-ui="${id}">${label}</button>`,cmd:(label,id)=>`<button data-action="${id}">${label}</button>`,field:(label,id)=>`<input id="${id}">${label}`,select:label=>label,kv:label=>label,icon:()=>''});
 const start=app.indexOf('function '+name+'('),end=app.indexOf('\nfunction ',start+1);vm.runInContext(app.slice(start,end),c);return c[name]();
}
test('driving and training describe operations while retaining actual capability limits',()=>{
 const motion=render('motion');assert.match(motion,/需要设备固件支持/);assert.match(motion,/停止.*手动驾驶/);assert.doesNotMatch(motion,/实验|原控制器|官方自定义灯语指令|全程无位移/);
 const joint=render('joints');assert.match(joint,/吊架或仿真/);assert.match(joint,/rig-confirmed/);assert.match(joint,/没有站立平衡保护/);
 const training=render('training','demo');assert.match(training,/演示训练.*模拟数据.*不生成可部署策略/);assert.doesNotMatch(training,/当前实现|已知不一致|先预览启动命令/);
});
test('ordinary navigation fallback has no second generic execution confirmation',async()=>{
 const calls=[],c=vm.createContext({S:{mode:'live'},$:()=>null,api:async(...args)=>calls.push(args),toast(){},fetchState:async()=>{},confirmAction(){throw Error('redundant confirmation');}});
 vm.runInContext(app.slice(app.indexOf('async function runRoute('),app.indexOf('async function nosAction(')),c);await c.runRoute('route');
 assert.equal(calls.length,1);assert.equal(calls[0][0],'/api/nav/start');
});
test('greeting status presents the action and preserves cancellation reasons',()=>{
 const c=vm.createContext({});vm.runInContext(app.slice(app.indexOf('function greetingStatusText('),app.indexOf('\nupdate=function',app.indexOf('function greetingStatusText('))),c);
 assert.equal(c.greetingStatusText({state:'motion',motion:'nod',message:'实验说明'}),'点头进行中');
 assert.match(c.greetingStatusText({state:'lighting',demo:true}),/演示/);
 assert.equal(c.greetingStatusText({state:'cancelled',message:'机器人仍在移动'}),'机器人仍在移动');
});
test('general pages omit project provenance and experimental product labels',()=>{
 for(const file of ['app.js','workflows.js','map_learning.js','vision_models.js']){
  const source=fs.readFileSync('platform_web/'+file,'utf8');assert.doesNotMatch(source,/已补齐原参考项目|使用你提供的|待实机验收|演示遥测用于验证交互|当前实现是|nav-tag[^>]*>LAB/);
 }
});
test('overview renders ordinary pose feedback and clears it when the platform connection drops',()=>{
 const readout={},calls=[];
 const sample={
  mode:'live',online:true,sdk:{connected:false,fresh:false},
  body_attitude:{roll:.1,pitch:-.2,yaw:.3},body_posture:{height:.22,motion_state:4},
  joints:[{name:'fl_hipy_joint',index:1,position:null,fresh:false}]
 };
 const c=vm.createContext({
  S:sample,page:'overview',stateUnavailable:false,$:()=>null,text:(id,value)=>readout[id]=value,
  fmt:(v,n)=>v.toFixed(n),viewer:{update:(...args)=>calls.push(args)}
 });
 vm.runInContext(app.slice(app.indexOf('function paintRobotAttitude('),app.indexOf('function setupViewer(')),c);
 c.paintRobotAttitude();
 assert.equal(readout['model-pill'],'已趴下');
 assert.match(readout['overview-source'],/0\.22 m · 横滚 5\.7° · 俯仰 -11\.5°.*腿部关节示意/);
 assert.equal(calls.at(-1)[3],sample.body_posture);
 sample.sdk={connected:true,fresh:true};
 sample.joints[0]={name:'fl_hipy_joint',index:1,position:-.4,fresh:true};
 c.paintRobotAttitude();
 assert.match(readout['overview-source'],/SDK 实测/);
 assert.equal(calls.at(-1)[0][0].position,-.4);
 c.stateUnavailable=true;
 c.paintRobotAttitude();
 assert.equal(readout['model-pill'],'平台连接中断');
 assert.equal(calls.at(-1)[2],null);
 assert.equal(calls.at(-1)[3],null);
 assert.equal(calls.at(-1)[0][0].position,undefined);
});
