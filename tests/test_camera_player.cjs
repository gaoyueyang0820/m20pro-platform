// Offline browser-state regression tests; no browser or robot connection.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const source=fs.readFileSync('platform_web/workspace.js','utf8');
const code=source.slice(source.indexOf('const cameraSessions='),source.indexOf('\nfunction cachedMapFor'));
function fixture(){
 const items=new Map(),revoked=[],calls=[],listeners={};
 const element=()=>({dataset:{},hidden:true,isConnected:true,naturalWidth:800,classList:{toggle(){},remove(){}},setAttribute(n){if(n==='hidden')this.hidden=true;},removeAttribute(n){if(n==='hidden')this.hidden=false;},querySelector(){return this.label||={textContent:''};},remove(){items.delete('#'+this.id);}});
 const img=element();img.parentElement={shown:true,getClientRects(){return this.shown?[{}]:[];},append(x){items.set('#'+x.id,x);}};items.set('#camera-front',img);items.set('#placeholder-front',element());
 let blobId=0,failDecode=false;
 const context=vm.createContext({console,performance:{now:()=>performance.now()+10000},AbortController,setTimeout,clearTimeout,setInterval,clearInterval,Image:class{decode(){return failDecode?Promise.reject(Error('bad jpeg')):Promise.resolve();}},
  URL:{createObjectURL(){return 'blob:'+(++blobId);},revokeObjectURL(u){revoked.push(u);}},document:{hidden:false,createElement:element,addEventListener(type,fn){(listeners[type]||=[]).push(fn);}},
  S:{mode:'live',online:true,ppe:{},cameras:{front:{fresh:false,width:800,height:600}}},page:'motion',driveView:'front',ui:{},
  $:key=>items.get(key),text:(key,value)=>{calls.push([key,value]);},api:async(...args)=>calls.push(args),
  fetch:async()=>{throw Error('network interrupted');}});
 vm.runInContext(code,context);
 return {context,img,items,revoked,calls,listeners,element,failDecode:v=>{failDecode=v;},sessions:()=>vm.runInContext('cameraSessions',context)};
}
const turn=()=>new Promise(r=>setImmediate(r));
function response(n){return {ok:true,headers:{get(k){return k==='X-Frame-Sequence'?String(n):'0.01';}},blob:async()=>({})};}

test('a complete new frame remains visible despite a stale 500ms status snapshot and request failure',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let count=0;
 f.context.fetch=async()=>{if(++count===1)return response(1);throw Error('temporary gap');};
 f.context.cameraOpen('front');await turn();await turn();
 assert.equal(f.img.src,'blob:1');assert.equal(f.items.get('#placeholder-front').hidden,true);
 assert.equal(f.items.get('#camera-notice-front').hidden,true);
 f.img.dataset.received=String(performance.now()+8500);f.context.paintCameraStatus();
 assert.equal(f.items.get('#placeholder-front').hidden,true);
 assert.equal(f.items.get('#camera-notice-front').hidden,false);
 assert.match(f.items.get('#camera-notice-front').textContent,/画面已暂停/);
 f.context.cameraClose('front');
});

test('bad JPEG never replaces last complete image',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let count=0;
 f.context.fetch=async()=>{if(++count===2)f.failDecode(true);return response(count);};
 f.context.cameraOpen('front');await turn();await turn();
 assert.equal(f.img.src,'blob:1');assert.ok(f.revoked.includes('blob:2'));
 assert.equal(f.sessions().get('front').sequence,1);
 f.context.cameraClose('front');
});

test('closing a camera during image decode prevents late display and releases the pending URL',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let finish;
 f.context.Image=class{decode(){return new Promise(r=>finish=r);}};
 f.context.fetch=async()=>response(1);f.context.cameraOpen('front');await turn();
 f.context.cameraClose('front');finish();await turn();
 assert.equal(f.img.hidden,true);assert.ok(f.revoked.includes('blob:1'));assert.equal(f.sessions().size,0);
});

test('reconnect invokes decoder recovery for only the selected view',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));f.context.cameraOpen=()=>{};
 await f.context.ui['drive-camera-retry']();
 assert.equal(f.calls[0][0],'/api/camera/reconnect');assert.equal(f.calls[0][1].camera,'front');
});

test('hidden camera pane aborts its request and does not poll until visible again',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let requests=0,signal;
 f.context.fetch=async(url,options)=>{requests++;signal=options.signal;return new Promise((resolve,reject)=>signal.addEventListener('abort',()=>reject(Object.assign(Error('aborted'),{name:'AbortError'}))));};
 f.context.cameraOpen('front');await turn();
 f.img.parentElement.shown=false;f.context.syncCameraVisibility();assert.equal(signal.aborted,true);
 await new Promise(r=>setTimeout(r,300));assert.equal(requests,1);
 f.img.parentElement.shown=true;f.context.syncCameraVisibility();
 await new Promise(r=>setTimeout(r,300));assert.equal(requests,2);
 assert.equal(f.sessions().size,1);
});

test('a frame decoding across hide and show is discarded and its URL released',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let finish,calls=0;
 f.context.Image=class{decode(){return new Promise(r=>finish=r);}};
 f.context.fetch=async()=>++calls===1?response(1):new Promise(()=>{});
 f.context.cameraOpen('front');await turn();
 f.context.document.hidden=true;f.context.syncCameraVisibility();
 f.context.document.hidden=false;f.context.syncCameraVisibility();finish();await turn();
 assert.equal(f.img.hidden,true);assert.ok(f.revoked.includes('blob:1'));
 assert.equal(f.sessions().get('front').sequence,-1);
});

function peopleFixture(){
 const f=fixture();const image=f.element();image.parentElement={shown:false,getClientRects(){return this.shown?[{}]:[];}};
 for(const name of ['people-preview','people-state','people-video-badge','people-camera','people-confidence','people-placeholder'])f.items.set('#'+name,name==='people-preview'?image:f.element());
 Object.assign(f.context,{page:'vision',vision:()=>'',render:()=>{},update:()=>{}});
 f.context.S.people_counts={enabled:true,paused:false,fresh:true,current:3,camera:'front',conf:.3};
 vm.runInContext(fs.readFileSync('platform_web/people_counting.js','utf8'),f.context);
 return {...f,peopleImage:image,people:()=>vm.runInContext('peoplePreview',f.context)};
}

test('state refresh never reopens hidden people preview; returning creates only one request',async t=>{
 const f=peopleFixture();t.after(()=>f.context.closePeoplePreview());let requests=0,signal;
 f.context.fetch=async(url,options)=>{requests++;signal=options.signal;return new Promise((resolve,reject)=>signal.addEventListener('abort',()=>reject(Error('aborted'))));};
 for(let i=0;i<5;i++)f.context.paintPeople();assert.equal(requests,0);
 f.peopleImage.parentElement.shown=true;f.context.paintPeople();f.context.paintPeople();assert.equal(requests,1);
 f.peopleImage.parentElement.shown=false;f.context.paintPeople();assert.equal(signal.aborted,true);assert.equal(f.people(),null);
 f.context.paintPeople();assert.equal(requests,1);
});

test('people pause and background hide abort preview without stopping the counting service',async t=>{
 const f=peopleFixture();t.after(()=>f.context.closePeoplePreview());const signals=[];
 f.context.fetch=async(url,options)=>{signals.push(options.signal);return new Promise((resolve,reject)=>options.signal.addEventListener('abort',()=>reject(Error('aborted'))));};
 f.peopleImage.parentElement.shown=true;f.context.paintPeople();
 f.context.S.people_counts.paused=true;f.context.paintPeople();assert.equal(signals[0].aborted,true);assert.equal(f.people(),null);
 f.context.S.people_counts.paused=false;f.context.paintPeople();assert.equal(signals.length,2);
 f.context.document.hidden=true;for(const fn of f.listeners.visibilitychange)fn();assert.equal(signals[1].aborted,true);
 assert.equal(f.context.S.people_counts.enabled,true);assert.ok(!f.calls.some(c=>String(c[0]).startsWith('/api/')));
});

test('closing people preview during decoding prevents late display and revokes its URL',async t=>{
 const f=peopleFixture();t.after(()=>f.context.closePeoplePreview());let finish;
 f.context.Image=class{decode(){return new Promise(r=>finish=r);}};f.context.fetch=async()=>response(1);
 f.peopleImage.parentElement.shown=true;f.context.paintPeople();await turn();
 f.peopleImage.parentElement.shown=false;f.context.paintPeople();finish();await turn();
 assert.equal(f.peopleImage.hidden,true);assert.ok(f.revoked.includes('blob:1'));assert.equal(f.people(),null);
});

test('sleep and camera power-off pause frame requests and show an actionable reason',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let requests=0;
 f.context.S.basic={Sleep:1};f.context.S.device={DevEnable:{Video:{Front:0,Back:0}}};
 f.context.fetch=async()=>{requests++;return new Promise(()=>{});};
 f.context.cameraOpen('front');f.context.paintCameraStatus();await turn();
 assert.equal(requests,0);assert.match(f.items.get('#placeholder-front').label.textContent,/休眠/);
 await f.context.ui['drive-camera-retry']();assert.ok(!f.calls.some(c=>c[0]==='/api/camera/reconnect'));
 f.context.S.basic.Sleep=0;f.context.S.device.DevEnable.Video.Front=1;f.context.paintCameraStatus();
 await new Promise(r=>setTimeout(r,300));assert.equal(requests,1);
});

test('camera-off feedback aborts the current preview without affecting the other view',async t=>{
 const f=fixture();t.after(()=>f.context.cameraClose('front'));let signal;
 f.context.fetch=async(url,options)=>{signal=options.signal;return new Promise((resolve,reject)=>signal.addEventListener('abort',()=>reject(Error('aborted'))));};
 f.context.S.basic={Sleep:0};f.context.S.device={DevEnable:{Video:{Front:1,Back:1}}};f.context.cameraOpen('front');await turn();
 f.context.S.device.DevEnable.Video.Front=0;f.context.paintCameraStatus();assert.equal(signal.aborted,true);
 assert.match(f.context.cameraUnavailableReason('front'),/关闭/);assert.equal(f.context.cameraUnavailableReason('rear'),'');
});

test('front and rear PPE previews request their own annotated feeds',t=>{
 const f=fixture(),rear=f.element(),requests=[];
 rear.parentElement=f.img.parentElement;f.items.set('#camera-rear',rear);
 f.items.set('#ppe-overlay',{checked:true});
 f.context.S.ppe={enabled:true,camera:'both',cameras:{front:{enabled:true},rear:{enabled:true}}};
 f.context.fetch=async(url,{signal})=>{
  requests.push(url);
  return new Promise((_,reject)=>signal.addEventListener('abort',()=>reject(Error('aborted'))));
 };
 t.after(()=>{f.context.cameraClose('front');f.context.cameraClose('rear');});
 f.context.cameraOpen('front');f.context.cameraOpen('rear');
 assert.deepEqual(requests,['/api/ppe/frame?camera=front&after=-1','/api/ppe/frame?camera=rear&after=-1']);
});
