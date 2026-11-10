const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
function setup(gos,nos={}){
 const messages={},calls=[];
 const field=values=>({value:values[0],dataset:{},options:values.map(value=>({value,disabled:false}))});
 const engine=field(['gos','native']),policy=field(['stop','finish','return','finish_return']);
 const context=vm.createContext({page:'navigation',navIntent:'goal',points:[],S:{server_instance:'a',connection_generation:1,mode:'live'},
  $:id=>({'#route-navigation-engine':engine,'#disconnect-policy':policy})[id],taskCollectorOptions:()=>[],
  document:{addEventListener(){}},
  text:(id,value)=>messages[id]=value,navigationPresentation:()=>null,render(){},update(){},
  api:async p=>{calls.push(p);return {...nos,gos};}});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../platform_web/disconnect_policy.js'),'utf8'),context);
 return {context,messages,calls,engine,policy};
}
test('navigation separates GOS status from its disconnect policy',async()=>{
 const f=setup({available:true});f.context.render();await new Promise(setImmediate);
 assert.equal(f.messages['navigation-executor-status'],'GOS 机载寻路已就绪');
 assert.match(f.messages['disconnect-policy-help'],/5 秒时停车/);
 assert.deepEqual(f.calls,['/api/nav/resident/capabilities']);
});
test('station action automatically selects GOS with stop on disconnect',()=>{
 const f=setup({available:true});f.engine.value='native';f.policy.value='finish';
 f.context.points=[{type:1,station_action:{camera:'front'}}];f.context.render();
 assert.equal(f.engine.value,'gos');assert.equal(f.policy.value,'stop');
 assert.equal(f.engine.disabled,true);
 f.context.points=[];f.context.update();assert.equal(f.engine.value,'native');
});
test('unavailable GOS reports the service status',async()=>{
 const f=setup({available:false,message:'请连接 GOS'});f.context.render();await new Promise(setImmediate);
 assert.equal(f.messages['navigation-executor-status'],'请连接 GOS');
});
test('charging waypoints show the dedicated docking workflow',()=>{
 const f=setup({available:true});f.context.navIntent='route';f.context.points=[{type:3}];f.context.render();
 assert.match(f.messages['navigation-executor-status'],/回桩流程/);
});
test('continuation choices require the native executor and its advertised policies',async()=>{
 const f=setup({available:true},{available:true,policies:['stop','return']});
 f.context.render();await new Promise(setImmediate);
 assert.equal(f.policy.options.find(option=>option.value==='return').disabled,true);
 f.engine.value='native';f.context.update();
 assert.equal(f.policy.options.find(option=>option.value==='return').disabled,false);
 assert.equal(f.policy.options.find(option=>option.value==='finish').disabled,true);
});
