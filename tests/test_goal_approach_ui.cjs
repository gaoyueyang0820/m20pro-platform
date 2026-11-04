const test=require('node:test');const assert=require('node:assert/strict');const vm=require('node:vm');const fs=require('node:fs');const path=require('node:path');
function setup(){
 const nodes=new Map();function node(tag){return {tag,attrs:{},children:[],textContent:'',setAttribute(k,v){this.attrs[k]=v;},append(...xs){this.children.push(...xs);for(const x of xs)if(x.id)nodes.set('#'+x.id,x);},remove(){nodes.delete('#'+this.id);}};}
 const svg=node('svg'),label=node('p');nodes.set('#nav-map',svg);nodes.set('#goal-plan-status',label);
 const p={goal:{x:1.25,y:2.25,yaw:0},goal_adjustment:{applied:true,requested:{x:1,y:2},distance_m:.354}};
 const context=vm.createContext({page:'navigation',navIntent:'goal',S:{nav:{status:'idle'}},goalPlan:p,valid:true,
  selectedMap:()=>({origin:[0,0,0],resolution:.1,height:80}),$:(id)=>nodes.get(id),
  paintGoalPlan(){label.textContent='参考路径';},drawMap(){},document:{createElementNS(ns,tag){return node(tag);}}});
 context.validGoalPlan=()=>context.valid;
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../platform_web/goal_approach.js'),'utf8'),context);return {context,nodes,p,svg,label};
}
test('adjusted endpoint and requested click are visibly distinct without changing the request',()=>{
 const {context,nodes,p,label}=setup();context.paintGoalPlan();context.drawMap();
 const group=nodes.get('#goal-approach-preview');assert.ok(group);
 const [line,marker]=group.children;assert.equal(line.attrs.x1,'10');assert.equal(line.attrs.y1,'60');
 assert.equal(marker.attrs.cx,'12.5');assert.equal(marker.attrs.cy,'57.5');assert.match(label.textContent,/0.35 米/);
 assert.deepEqual(p.goal_adjustment.requested,{x:1,y:2});assert.equal(marker.attrs['aria-label'],'计划停车位置');
});
test('invalidated preview removes the old adjusted marker',()=>{
 const {context,nodes}=setup();context.drawMap();assert.ok(nodes.get('#goal-approach-preview'));
 context.valid=false;context.drawMap();assert.equal(nodes.get('#goal-approach-preview'),undefined);
});
test('an executing plan shows its actual endpoint rather than a newly edited preview',()=>{
 const {context,nodes}=setup();context.S.nav={status:'running',plan:{goal:{x:4,y:3,yaw:0},goal_adjustment:{applied:true,requested:{x:4.3,y:3},distance_m:.3}}};
 context.drawMap();assert.equal(nodes.get('#goal-approach-preview').children[1].attrs.cx,'40');
});
