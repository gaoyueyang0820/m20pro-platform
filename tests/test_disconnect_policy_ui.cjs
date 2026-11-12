const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
function setup(gos,nos={},online=true){
 const messages={},calls=[],events={};
 const field=values=>({value:values[0],dataset:{},options:values.map(value=>({value,disabled:false}))});
 const engine=field(['gos','native']),policy=field(['stop','finish','return','finish_return']);
 const context=vm.createContext({page:'navigation',navIntent:'goal',points:[],S:{server_instance:'a',connection_generation:1,mode:'live',online},
  $:id=>({'#route-navigation-engine':engine,'#disconnect-policy':policy})[id],taskCollectorOptions:()=>[],
  document:{addEventListener:(name,fn)=>events[name]=fn},
  text:(id,value)=>messages[id]=value,navigationPresentation:()=>null,render(){},update(){},
  api:async p=>{calls.push(p);return {...nos,gos};}});
 const workflows=fs.readFileSync(path.join(__dirname,'../platform_web/workflows.js'),'utf8');
 vm.runInContext(workflows.slice(workflows.indexOf('function patrolNeedsGos'),workflows.indexOf('function navigationIntent')),context);
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../platform_web/disconnect_policy.js'),'utf8'),context);
 return {context,messages,calls,engine,policy,change:id=>events.change({target:{id,value:engine.value,matches:()=>true}})};
}
test('navigation separates GOS status from its disconnect policy',async()=>{
 const f=setup({available:true});f.context.render();await new Promise(setImmediate);
 assert.equal(f.messages['navigation-executor-status'],'GOS 寻路服务已连接');
 assert.match(f.messages['disconnect-policy-help'],/5 秒后请求停车.*原厂导航/);
 assert.deepEqual(f.calls,['/api/nav/resident/capabilities']);
});
test('ball camera station action automatically selects GOS with stop on disconnect',()=>{
 const f=setup({available:true});f.engine.value='native';f.policy.value='finish';
 f.context.points=[{type:1,station_action:{camera:'ball'}}];f.context.render();
 assert.equal(f.engine.value,'gos');assert.equal(f.policy.value,'stop');
 assert.equal(f.engine.disabled,true);
 f.context.points=[];f.context.update();assert.equal(f.engine.value,'native');assert.equal(f.policy.value,'finish');
});
test('front camera station counting allows original navigation and stop on disconnect',()=>{
 const f=setup({available:true},{available:true,policies:['return']});
 f.engine.value='native';f.policy.value='return';
 f.context.points=[{type:1,station_action:{camera:'front',pan:'none'}}];
 f.context.render();
 assert.equal(f.engine.disabled,false);
 assert.equal(f.engine.value,'native');
 assert.equal(f.policy.value,'stop');
 assert.match(f.messages['disconnect-policy-help'],/到点停稳后由电脑统计/);
});
test('front or rear people counts preserve native navigation and its disconnect policy',()=>{
 const f=setup({available:true},{available:true,policies:['return']});
 f.engine.value='native';f.policy.value='return';
 const collector={kind:'people_count',camera:'front'};
 f.context.taskCollectorOptions=()=>[collector];
 f.context.render();
 assert.equal(f.engine.disabled,false);
 assert.equal(f.policy.value,'return');
 assert.match(f.messages['disconnect-policy-help'],/人数由电脑采集/);
 collector.camera='ball';f.context.update();
 assert.equal(f.engine.value,'gos');assert.equal(f.policy.value,'stop');
 collector.camera='rear';f.context.update();
 assert.equal(f.engine.value,'native');assert.equal(f.policy.value,'return');
});
test('unavailable GOS reports the service status',async()=>{
 const f=setup({available:false,message:'请连接 GOS'});f.context.render();await new Promise(setImmediate);
 assert.equal(f.messages['navigation-executor-status'],'请连接 GOS');
});
test('charging waypoints show the dedicated docking workflow',()=>{
 const f=setup({available:true});f.context.navIntent='route';f.context.points=[{type:3}];f.context.render();
 assert.match(f.messages['navigation-executor-status'],/回桩流程/);
});
test('native continuation options remain editable while the robot is offline',()=>{
 const f=setup({available:false},{},false);f.context.render();
 assert.equal(f.policy.options.find(option=>option.value==='return').disabled,true);
 f.engine.value='native';f.change('route-navigation-engine');
 assert.ok(f.policy.options.every(option=>!option.disabled));
 f.policy.value='finish_return';f.change('disconnect-policy');f.context.update();
 assert.equal(f.policy.value,'finish_return');assert.match(f.messages['disconnect-policy-help'],/可先保存路线/);
 assert.deepEqual(f.calls,[]);
});
test('show readiness for the selected native policy without blocking its configuration',async()=>{
 const f=setup({available:true},{available:true,policies:['stop','return']});
 f.context.render();await new Promise(setImmediate);
 assert.equal(f.policy.options.find(option=>option.value==='return').disabled,true);
 f.engine.value='native';f.change('route-navigation-engine');assert.ok(f.policy.options.every(option=>!option.disabled));
 f.policy.value='return';f.change('disconnect-policy');assert.match(f.messages['disconnect-policy-help'],/按所选策略执行/);
 f.policy.value='finish';f.change('disconnect-policy');assert.match(f.messages['disconnect-policy-help'],/可先保存路线/);
 f.engine.value='gos';f.change('route-navigation-engine');assert.equal(f.policy.value,'stop');
 f.engine.value='native';f.change('route-navigation-engine');assert.equal(f.policy.value,'finish');
});
