'use strict';
// Display the actual body-checked endpoint separately from the operator's click.
(()=>{
 function plan(){
  if(page!=='navigation'||navIntent!=='goal')return null;
  const p=S?.nav?.status==='running'?S.nav.plan:validGoalPlan()?goalPlan:null;
  return p?.goal_adjustment?.applied?p:null;
 }
 const previousPaint=paintGoalPlan;
 paintGoalPlan=function(...args){
  previousPaint.apply(this,args);const p=plan(),el=$('#goal-plan-status');
  if(p&&el)el.textContent+=' · 将停在目标附近 '+Number(p.goal_adjustment.distance_m).toFixed(2)+' 米处（图中橙色终点）';
 };
 const previousDraw=drawMap;
 drawMap=function(...args){
  previousDraw.apply(this,args);$('#goal-approach-preview')?.remove();
  const p=plan(),m=selectedMap(),svg=$('#nav-map');if(!p||!m||!svg)return;
  const target=p.goal_adjustment.requested,actual=p.goal;
  if(![target?.x,target?.y,actual?.x,actual?.y,m.resolution].every(Number.isFinite)||m.resolution<=0)return;
  const xy=p=>[(p.x-m.origin[0])/m.resolution,m.height-(p.y-m.origin[1])/m.resolution];
  const a=xy(actual),t=xy(target),ns='http://www.w3.org/2000/svg';
  const group=document.createElementNS(ns,'g');group.id='goal-approach-preview';group.setAttribute('pointer-events','none');
  const line=document.createElementNS(ns,'line');
  for(const [key,value] of Object.entries({x1:t[0],y1:t[1],x2:a[0],y2:a[1],stroke:'#c78b26','stroke-width':2,'stroke-dasharray':'4 3'}))line.setAttribute(key,String(value));
  const marker=document.createElementNS(ns,'circle');
  for(const [key,value] of Object.entries({cx:a[0],cy:a[1],r:Math.max(3,.13/m.resolution),fill:'#e5a733',stroke:'#fff','stroke-width':1.5,'aria-label':'实际可到达终点'}))marker.setAttribute(key,String(value));
  const title=document.createElementNS(ns,'title');title.textContent='实际终点：距所选位置 '+Number(p.goal_adjustment.distance_m).toFixed(2)+' 米，已核验完整机身空间';
  marker.append(title);group.append(line,marker);svg.append(group);
 };
})();
