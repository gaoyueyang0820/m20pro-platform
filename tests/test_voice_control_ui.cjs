const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
function setup(){
 const events={},windowEvents={},timers=new Map(),requests=[],instances=[];let now=1000;
 const button={classList:{add(){},remove(){}},setAttribute(){},addEventListener(name,fn){this[name]=fn;}};
 const status={textContent:'',hidden:true};
 class Recognition{constructor(){instances.push(this);}start(){this.started=true;}abort(){this.aborted=true;}}
 const document={visibilityState:'visible',addEventListener(name,fn){events[name]=fn;}};
 const window={SpeechRecognition:Recognition,addEventListener(name,fn){windowEvents[name]=fn;}};
 const context=vm.createContext({window,document,page:'motion',S:{mode:'live',ip:'192.0.2.10'},authPending:false,
  $:selector=>selector==='#voice-toggle'?button:status,Date:{now:()=>now},
  setTimeout(fn){const id=timers.size+1;timers.set(id,fn);return id;},clearTimeout(id){timers.delete(id);},
  api:async(url,body)=>{requests.push({url,body});}});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../platform_web/voice_control.js'),'utf8'),context);
 const say=(recognition,text)=>recognition.onresult?.({resultIndex:0,results:[Object.assign([{transcript:text}],{isFinal:true})]});
 return {context,button,status,events,windowEvents,requests,instances,timers,say,advance:ms=>{now+=ms;}};
}
test('one exact command consumes listening and never automatically restarts',async()=>{
 const s=setup();s.button.click();const r=s.instances[0],late=r.onresult;await s.say(r,'站起来！');
 assert.equal(s.requests.length,1);assert.equal(s.requests[0].body.action,'stand');assert.ok(r.aborted);
 await late({resultIndex:0,results:[Object.assign([{transcript:'趴下'}],{isFinal:true})]});assert.equal(s.requests.length,1);
 assert.equal(s.instances.length,1);
});
test('negation, questions, quoted speech and combined actions never issue commands',async()=>{
 for(const text of ['不要站起来','别趴下','可以站起来吗','他说站起来','站起来然后趴下','坐下','躺下']){
  const s=setup();s.button.click();await s.say(s.instances[0],text);assert.equal(s.requests.length,0,text);
 }
});
test('hidden page, lost focus, navigation and stop invalidate delayed recognition',async()=>{
 for(const type of ['visibility','blur','hashchange','manual']){
  const s=setup();s.button.click();const r=s.instances[0],late=r.onresult;
  if(type==='visibility'){s.context.document.visibilityState='hidden';s.events.visibilitychange();}
  else if(type==='manual')s.events.click({target:{closest:()=>({})}});
  else s.windowEvents[type]();
  await late({resultIndex:0,results:[Object.assign([{transcript:'站立'}],{isFinal:true})]});assert.equal(s.requests.length,0,type);
 }
});
test('connection changes and timeout cannot authorize an old utterance',async()=>{
 for(const change of ['ip','mode','timeout']){
  const s=setup();s.button.click();const r=s.instances[0];
  if(change==='timeout')s.advance(15001);else s.context.S[change]=change==='mode'?'demo':'192.0.2.20';
  await s.say(r,'站立');assert.equal(s.requests.length,0,change);
 }
});
test('recognition ending or failing does not restart the microphone',()=>{
 for(const action of ['onend','onerror']){const s=setup();s.button.click();s.instances[0][action]({error:'network'});assert.ok(s.instances[0].aborted);assert.equal(s.instances.length,1);assert.equal(s.timers.size,0);}
});
