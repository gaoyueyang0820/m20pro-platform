// UI task lifecycle only. No model, HTTP service, map or robot connection.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
function fixture(){
 const result={hidden:true,innerHTML:'',scrollIntoView(){}},button={disabled:false};
 const c=vm.createContext({vision:()=>'',update:()=>{},S:{mode:'demo',ppe:{}},page:'vision',ui:{},
  $:s=>s==='#model-check-result'?result:null,$$:()=>[button],fmt:v=>String(v),kv:(k,v)=>`${k}: ${v}`,toast(){},
  esc:v=>String(v??'').replaceAll('<','&lt;'),job:async()=>({infer_ms:10,counts:{person:3},image:'/fixture.jpg'})});
 vm.runInContext(fs.readFileSync('platform_web/vision_models.js','utf8'),c);return {c,result,button};
}
test('model result is a sibling of tabs, not inside the hidden models pane',()=>{
 const f=fixture();let slot='',sibling='';
 f.c.S.vision_models=[];f.c.operationPanel=()=>'';
 f.c.document={createElement:()=>({querySelector:s=>s.includes('data-vision-pane')?{set innerHTML(v){slot=v;}}:null,insertAdjacentHTML:(where,v)=>{sibling=v;},innerHTML:''})};
 f.c.vision();assert.ok(!slot.includes('model-check-result'));assert.ok(sibling.includes('model-check-result'));
});
test('pending work remains disabled across status updates and result is visible on people tab',async()=>{
 const f=fixture();let resolve;f.c.job=()=>new Promise(r=>resolve=r);
 const pending=f.c.ui['check-counting']();f.c.update();
 assert.equal(f.button.disabled,true);assert.equal(f.result.hidden,false);assert.match(f.result.innerHTML,/正在检查模型/);
 resolve({infer_ms:12,counts:{person:3},image:'/sample.jpg'});await pending;
 assert.equal(f.button.disabled,false);assert.match(f.result.innerHTML,/人数统计模型 · 本机检查完成/);assert.match(f.result.innerHTML,/人员: 3/);
});
test('completed check survives leaving and returning to the vision page',async()=>{
 const f=fixture();let resolve;f.c.job=()=>new Promise(r=>resolve=r);
 const pending=f.c.ui['check-counting']();f.c.page='motion';resolve({infer_ms:12,counts:{},image:'/sample.jpg'});await pending;
 f.result.innerHTML='';f.result._markup=null;f.c.page='vision';f.c.update();
 assert.match(f.result.innerHTML,/本机检查完成/);assert.match(f.result.innerHTML,/未检测到目标/);
});
test('failure shows its reason and permits a later retry',async()=>{
 const f=fixture();f.c.job=async()=>{throw Error('模型文件缺失');};await f.c.ui['check-counting']();
 assert.equal(f.button.disabled,false);assert.match(f.result.innerHTML,/模型文件缺失/);assert.match(f.result.innerHTML,/重试/);
});
