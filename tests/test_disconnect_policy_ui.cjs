const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
function setup(available=true){
 const handlers={},calls=[],select={value:'stop',options:['stop','finish','return','finish_return'].map(value=>({value,disabled:false}))};
 const context=vm.createContext({page:'navigation',navIntent:'goal',routeId:null,points:[],S:{server_instance:'a',connection_generation:1,mode:'live',nav:{status:'idle'},routes:[]},
  window:{M20NavigationPreparation:{cancel(){}}},document:{addEventListener(name,fn){handlers[name]=fn;}},
  $:selector=>selector==='#disconnect-policy'?select:null,text(){},navPage:()=>'<aside></aside>',goalKey:()=> 'goal',routeOptions:()=>({nav_mode:1}),
  pointNavigationEngine:'native',invalidateGoalPlan(){},refreshGoalPlan(){},navigationPresentation:()=>null,
  render(){},update(){},api:async(p,b)=>{calls.push([p,b]);return {protocol:1,available};}});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../platform_web/disconnect_policy.js'),'utf8'),context);
 return {context,calls,select,handlers};
}
test('all loss policies are visible and stop remains default',()=>{
 const {context,select}=setup();const markup=context.navPage();for(const p of ['stop','finish','return','finish_return'])assert.ok(markup.includes('value="'+p+'"'));
 assert.equal(select.value,'stop');assert.equal(context.routeOptions().disconnect_policy,'stop');
});
test('unavailable onboard capability prevents non-default start instead of silently downgrading',async()=>{
 const s=setup(false);s.context.render();await new Promise(setImmediate);s.select.value='return';
 await assert.rejects(s.context.api('/api/nav/goal',{engine:'native'}),/尚未就绪/);
 assert.ok(!s.calls.some(([p])=>p==='/api/nav/goal'));
});
test('available capability forwards selected policy and rejects desktop execution',async()=>{
 const s=setup(true);s.context.render();await new Promise(setImmediate);s.select.value='finish_return';
 await s.context.api('/api/nav/goal',{engine:'native'});assert.equal(s.calls.at(-1)[1].disconnect_policy,'finish_return');
 await assert.rejects(s.context.api('/api/nav/goal',{engine:'platform'}),/机器人端寻路/);
});
test('route options and goal identity include loss policy',()=>{
 const s=setup();const before=s.context.goalKey();s.select.value='finish';assert.notEqual(s.context.goalKey(),before);assert.equal(s.context.routeOptions().disconnect_policy,'finish');
});
