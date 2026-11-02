const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync('platform_web/hub.js','utf8');
function fixture(){
 const shown=[],jobs=[],listeners={};
 const c=vm.createContext({page:'maps',S:{mode:'demo',ip:'fixture'},cloudSource:null,cloudViewer:{pose(){}},cloudViewGeneration:0,
  ui:{},showCloud:d=>shown.push(d),text(){},$:s=>({value:'/fixture'}),
  job:(path,data)=>new Promise((resolve,reject)=>jobs.push({data,resolve,reject})),api:async path=>({name:path}),
  document:{addEventListener:(name,fn)=>{(listeners[name]||=[]).push(fn);}}});
 vm.runInContext(source.slice(source.indexOf('async function previewCloud('),source.indexOf("ui['map-library-refresh']")),c);
 vm.runInContext(source.slice(source.indexOf('let liveCloudTopic='),source.indexOf('const oldSensorsPage=')),c);
 return {c,shown,jobs};
}
const turn=()=>new Promise(setImmediate);
test('slower earlier map cannot overwrite the last selected map',async()=>{
 const f=fixture();const first=f.c.previewCloud('map','first'),second=f.c.previewCloud('map','second');
 f.jobs[1].resolve({cloud_id:'second'});await second;f.jobs[0].resolve({cloud_id:'first'});await first;
 assert.deepEqual(f.shown.map(x=>x.name),['/api/cloud/data/second']);assert.equal(f.c.cloudSource.key,'second');
});
test('leaving maps while reading a preview prevents a late display',async()=>{
 const f=fixture();const pending=f.c.previewCloud('map','first');f.c.page='motion';f.jobs[0].resolve({cloud_id:'first'});await pending;assert.equal(f.shown.length,0);
});
test('stopping live viewing invalidates a pending saved-map preview',async()=>{
 const f=fixture();const pending=f.c.previewCloud('map','first');f.c.ui['cloud-live-stop']();f.jobs[0].resolve({cloud_id:'first'});await pending;assert.equal(f.shown.length,0);
});
test('late failure for an obsolete map does not report an error over the latest map',async()=>{
 const f=fixture();const first=f.c.previewCloud('map','first'),second=f.c.previewCloud('map','second');
 f.jobs[1].resolve({cloud_id:'second'});await second;f.jobs[0].reject(Error('old map unavailable'));await first;assert.equal(f.shown.length,1);
});
test('switching runtime mode rejects an old map result',async()=>{
 const f=fixture();const pending=f.c.previewCloud('map','first');f.c.S.mode='live';f.jobs[0].resolve({cloud_id:'first'});await pending;assert.equal(f.shown.length,0);
});

test('obsolete background job stops polling and cannot publish operation status',async()=>{
 let current=true,resolve,calls=0;const progress=[];
 const c=vm.createContext({api:()=>{calls++;return new Promise(r=>resolve=r);},setOperation:(...args)=>progress.push(args),setTimeout});
 vm.runInContext(source.slice(source.indexOf('async function waitJob('),source.indexOf('async function job(')),c);
 const pending=c.waitJob({job_id:'old'},'fixture',()=>current,true);current=false;
 resolve({jobs:{status:'completed',result:{cloud_id:'old'},progress:1}});assert.equal(await pending,null);assert.equal(calls,1);assert.equal(progress.length,0);
});
test('current background job still returns its result and reports progress',async()=>{
 const progress=[];const c=vm.createContext({api:async()=>({jobs:{status:'completed',result:{cloud_id:'new'},progress:1,message:'ready'}}),setOperation:(...args)=>progress.push(args),setTimeout});
 vm.runInContext(source.slice(source.indexOf('async function waitJob('),source.indexOf('async function job(')),c);
 const result=await c.waitJob({job_id:'new'},'fixture');assert.equal(result.cloud_id,'new');assert.equal(progress.length,1);
});
test('API forwards abort signal and keeps cancelled read errors quiet',async()=>{
 const app=fs.readFileSync('platform_web/app.js','utf8');const begin=app.indexOf('async function api(');const code=app.slice(begin,app.indexOf('\n',begin));let signal,toastCount=0;
 const c=vm.createContext({fetch:async(path,options)=>{signal=options.signal;return {ok:true,json:async()=>({ok:true})};},toast:()=>toastCount++});vm.runInContext(code,c);
 const controller=new AbortController();await c.api('/fixture',{},true,{signal:controller.signal});assert.equal(signal,controller.signal);
 c.fetch=async()=>{throw Object.assign(Error('aborted'),{name:'AbortError'});};await assert.rejects(c.api('/fixture',{},true,{signal:controller.signal}));assert.equal(toastCount,0);
});
