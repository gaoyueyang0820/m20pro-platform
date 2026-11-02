// Synthetic display state only: no server, robot, motion integration or timers.
const {test}=require('node:test');const assert=require('node:assert/strict');
const fs=require('node:fs'),vm=require('node:vm');
function fixture(){
 let now=0,markup='';const nodes=new Map();
 const element=()=>({hidden:false,textContent:'',attrs:{},setAttribute(k,v){this.attrs[k]=v;}});
 const circle=element(),title=element(),path=element(),marker=element();marker.querySelector=q=>q==='circle'?circle:q==='title'?title:null;
 const svg={querySelector:q=>q==='#auto-robot-marker'&&markup.includes('id="auto-robot-marker"')?marker:q==='#auto-planned-path'&&markup.includes('id="auto-planned-path"')?path:null};
 Object.defineProperty(svg,'innerHTML',{get:()=>markup,set:v=>{markup=v;}});
 nodes.set('#auto-view',svg);for(const id of ['auto-empty','auto-observation-status','auto-navigation-status'])nodes.set('#'+id,element());
 const c=vm.createContext({S:{mode:'live',ip:'fixture',server_instance:'fixture',mapping:true,online:true,autonomy:{active:true}},performance:{now:()=>now},
  $:selector=>nodes.get(selector),text:(id,text)=>{nodes.get('#'+id).textContent=text;},fmt:(v,n=1)=>Number(v).toFixed(n)});
 const source=fs.readFileSync('platform_web/autonomy.js','utf8');
 vm.runInContext('let autoObservationClock=null,autoStateReceipt=null;'+source.slice(source.indexOf('function autoMapObservation('),source.indexOf('function autoDrawVision(')),c);
 const map={floor:[[-4,-4],[0,0],[4,4]],obstacles:[[4,0]],path:[[0,0],[3,0]],resolution:.2,pose:[0,0,0],capture_id:'capture',
  observation:{live:true,age:.1,stamp:100,frame:'slam_session',capture_id:'capture',message:'手动接管中，位置实时更新；背景为最近探索观测'}};
 return {c,map,nodes,circle,title,path,marker,advance:ms=>{now+=ms;},html:()=>markup};
}
test('paused mapping accepts fresh same-session measured pose without displaying a driving path',()=>{
 const f=fixture();f.c.S.autonomy.active=false;f.map.pose=[2,1,Math.PI/2];f.c.autoDrawMap(f.map);
 assert.equal(f.marker.attrs['data-observation-state'],'live');assert.equal(f.path.attrs.display,'none');
 assert.match(f.html(),/rotate\(-90\)/);assert.match(f.nodes.get('#auto-observation-status').textContent,/实[测时]|更新/);
});
test('saved map is historical and never substitutes global navigation coordinates',()=>{
 const f=fixture();f.c.S.mapping=false;f.c.S.autonomy.active=false;f.c.S.pose={PosX:999,PosY:777,Location:1};
 f.map.observation.live=false;f.map.observation.message='采集已结束，显示最后观测';f.map.pose=[.4,.2,.1];f.c.autoDrawMap(f.map);
 assert.equal(f.marker.attrs['data-observation-state'],'history');assert.match(f.title.textContent,/非当前位置/);
 assert.doesNotMatch(f.html(),/999|777/);assert.match(f.nodes.get('#auto-navigation-status').textContent,/未就绪/);
});
test('wrong frame or capture cannot be painted as a current SLAM position',()=>{
 for(const key of ['frame','capture_id']){
  const f=fixture();f.map.observation[key]='other';f.c.autoDrawMap(f.map);
  assert.doesNotMatch(f.html(),/id="auto-robot-marker"/);assert.match(f.nodes.get('#auto-observation-status').textContent,/坐标系不匹配/);
 }
});
test('waiting for a new task first scan clears old map and marker',()=>{
 const f=fixture();f.c.autoDrawMap(f.map);assert.ok(f.html());f.c.autoDrawMap(null);
 assert.equal(f.html(),'');assert.equal(f.nodes.get('#auto-empty').hidden,false);
 assert.match(f.nodes.get('#auto-observation-status').textContent,/等待本轮/);
});
test('a stalled browser status stream ages the last position instead of staying live',()=>{
 const f=fixture();f.c.autoDrawMap(f.map);f.advance(2500);f.c.autoPaintMapObservation(f.map);
 assert.equal(f.marker.attrs['data-observation-state'],'history');assert.equal(f.path.attrs.display,'none');
 assert.match(f.nodes.get('#auto-observation-status').textContent,/过期/);
});
test('same source stamp cannot become younger just because another status packet arrives',()=>{
 const f=fixture();f.c.autoDrawMap(f.map);f.advance(2300);f.c.S={...f.c.S};f.c.autoPaintMapObservation(f.map);
 assert.equal(f.marker.attrs['data-observation-state'],'history');
 f.map.observation.stamp=101;f.c.autoDrawMap(f.map);assert.equal(f.marker.attrs['data-observation-state'],'live');
});
test('legacy metadata is labelled unverified during activity and historical after pause',()=>{
 const f=fixture();delete f.map.observation;f.c.autoDrawMap(f.map);
 assert.equal(f.marker.attrs['data-observation-state'],'unverified');assert.equal(f.path.attrs.display,'none');
 f.c.S.autonomy.active=false;f.c.autoDrawMap(f.map);assert.equal(f.marker.attrs['data-observation-state'],'history');
});
test('manual movement beyond old scan bounds remains within the drawn viewport',()=>{
 const f=fixture();f.c.S.autonomy.active=false;f.map.pose=[15,20,Math.PI];f.c.autoDrawMap(f.map);
 const match=f.html().match(/id="auto-robot-marker" transform="translate\(([-\d.]+),([-\d.]+)\)/);
 assert.ok(match);assert.ok(+match[1]>=16&&+match[1]<=664);assert.ok(+match[2]>=16&&+match[2]<=444);
});
