const test=require('node:test');
const assert=require('node:assert/strict');
const {navigationPresentation:p,previousStopPresentation:previous,navigationFeedback:feedback}=require('../platform_web/navigation_status.js');
test('accepted or preparing tasks are not labelled as moving',()=>{
 assert.equal(p({status:'running',phase:'preparing'}).label,'准备导航');
 assert.match(p({status:'running',execution_state:'awaiting_progress'}).title,/等待机器人行动/);
});
test('zero-progress running task exposes actual waiting state',()=>{
 const r=p({status:'running',execution_state:'waiting_progress',warning:'12秒没有位移'});
 assert.equal(r.label,'等待实际进展');assert.equal(r.detail,'12秒没有位移');
});
test('missing or rejected cancel acknowledgement never says stopped',()=>{
 for(const phase of ['cancel_unconfirmed','cancel_rejected']){
  const r=p({status:'cancelling',phase});assert.equal(r.label,'停止未确认');assert.doesNotMatch(r.title,/已停止/);
 }
});
test('arrival awaiting cancellation differs from completed stop',()=>{
 assert.match(p({status:'cancelling',stop_reason:'route_complete'}).title,/确认停车/);
 assert.equal(p({status:'completed'}).label,'已完成');
});
test('cancelled preparation does not retain stale preparing text',()=>{
 const r=p({status:'cancelled',phase:'stopped',message:'正在确认导航模式'});
 assert.equal(r.title,'任务已取消');assert.doesNotMatch(r.detail,/确认导航模式/);
});
test('navigation failures show the actual reason',()=>{
 const r=p({status:'failed',phase:'preparation_failed',error:'步态尚未确认'});
 assert.equal(r.title,'导航尚未开始');assert.equal(r.detail,'步态尚未确认');
});
test('switching to demo does not erase an unconfirmed old robot stop',()=>{
 assert.ok(previous({state:'unconfirmed'}));assert.ok(previous({state:'active_after_reconnect'}));
 assert.equal(previous({state:'idle_confirmed'}),null);assert.equal(previous(null),null);
});
test('feedback card and map badge use the same actual state, not bare running',()=>{
 const r=feedback({status:'running',execution_state:'waiting_progress',warning:'<untrusted>',completed:0,route:{points:[{}]}});
 assert.equal(r.label,'等待实际进展');assert.match(r.html,/等待实际进展/);assert.doesNotMatch(r.html,/执行中/);
 assert.match(r.html,/&lt;untrusted&gt;/);assert.match(r.html,/native-nav-diagnostics/);
});
test('platform executor reports trajectory progress and measured stop instead of native receipts',()=>{
 const n=p({engine:'platform',status:'running',execution_state:'moving',distance_remaining:2.3,replans:1,message:'侧移绕开障碍'});
 assert.equal(n.label,'平台自主寻路');assert.match(n.detail,/2.3 米.*1 次/);assert.match(n.detail,/侧移/);
 const s=p({engine:'platform',status:'cancelling'});assert.match(s.detail,/实际停稳/);assert.doesNotMatch(s.detail,/对应回包/);
});
