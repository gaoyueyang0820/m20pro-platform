const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
function setup(gos){
 const messages={},calls=[];
 const context=vm.createContext({page:'navigation',navIntent:'goal',points:[],S:{server_instance:'a',connection_generation:1,mode:'live'},
  text:(id,value)=>messages[id]=value,navigationPresentation:()=>null,render(){},update(){},
  api:async p=>{calls.push(p);return {gos};}});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../platform_web/disconnect_policy.js'),'utf8'),context);
 return {context,messages,calls};
}
test('navigation displays GOS and stop on disconnect as status',async()=>{
 const f=setup({available:true});f.context.render();await new Promise(setImmediate);
 assert.equal(f.messages['navigation-executor-status'],'GOS 机载寻路 · 持续断联停车');
 assert.deepEqual(f.calls,['/api/nav/resident/capabilities']);
});
test('unavailable GOS reports the service status',async()=>{
 const f=setup({available:false,message:'请连接 GOS'});f.context.render();await new Promise(setImmediate);
 assert.equal(f.messages['navigation-executor-status'],'请连接 GOS');
});
test('charging waypoints show the dedicated docking workflow',()=>{
 const f=setup({available:true});f.context.navIntent='route';f.context.points=[{type:3}];f.context.render();
 assert.match(f.messages['navigation-executor-status'],/回桩流程/);
});
