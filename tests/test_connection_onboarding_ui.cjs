// Production onboarding handlers with local replies; no network discovery or deployment.
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const hub=fs.readFileSync('platform_web/hub.js','utf8'),source=fs.readFileSync('platform_web/connections.js','utf8');
const tick=()=>new Promise(setImmediate);
function fixture(deployed=false){
 const calls=[],dialogs=[],events={},nodes={};
 for(const id of ['connections-profile-form','setup-user','setup-password','setup-admin-password','nos-user','hub-aos-ssh-user','quick-aos-ip','hub-aos-ip'])nodes['#'+id]={value:id==='setup-user'?'operator':'',autocomplete:''};
 nodes['#modal']={open:false,close(){this.open=false;}};nodes['[data-ui="setup-deploy"]']={disabled:false};nodes['[data-ui="discover-robot"]']={disabled:false};
 const targets=Object.fromEntries(['nos','gos','sdk','train'].map(role=>[role,{host:'fixture-'+role,user:'user',ssh_port:22,deployed,via_aos:true}]));
 const c=vm.createContext({console,S:{server_instance:'server',connection_generation:1},page:'settings',connections:{aos_ip:'fixture-aos',targets},setupRole:null,setupGeneration:0,mapDialogGeneration:0,ui:{},
  roleNames:{nos:'NOS',gos:'GOS',sdk:'SDK',train:'训练'},$:s=>nodes[s],esc:String,field:(label,id,value='')=>`<label>${label}<input id="${id}" value="${value}"></label>`,btn:(label,id)=>`<button data-ui="${id}">${label}</button>`,
  text(){},toast:(...args)=>calls.push(['toast',...args]),modal:(title,body)=>{c.mapDialogGeneration++;nodes['#modal'].open=true;dialogs.push({title,body});},
  readConnections:()=>({targets:{nos:{user:nodes['#nos-user'].value}}}),loadConnections:async()=>calls.push(['load']),connectionCenter:()=>calls.push(['paint']),
  api:async(path,data)=>{calls.push(['api',path,data]);return {ok:true};},job:async(path,data,label,isCurrent)=>{calls.push(['job',path,data,isCurrent]);return {ok:true};},
  document:{addEventListener:(name,fn)=>(events[name]||=[]).push(fn)}});
 vm.runInContext(hub.slice(hub.indexOf('let setupInstallIntent='),hub.indexOf("mapsPage=function(){",hub.indexOf('let setupInstallIntent='))),c);
 vm.runInContext(source.slice(source.indexOf('let connectionDiscoveryBusy=')),c);
 return {c,calls,dialogs,nodes,events};
}
test('first installation has one credentials form and no separate fingerprint ceremony',async()=>{
 const f=fixture();await f.c.setupWizard('nos');assert.equal(f.dialogs.length,1);
 assert.match(f.dialogs[0].body,/设备账号/);assert.match(f.dialogs[0].body,/安装并连接/);
 assert.doesNotMatch(f.dialogs[0].body,/指纹|trust-confirm|setup-scan|setup-trust|setup-confirm/);
 assert.equal(f.calls.some(r=>r[0]==='job'||r[1]?.includes('/pair')||r[1]?.includes('/scan')),false);
 f.nodes['#setup-password'].value='synthetic';await f.c.ui['setup-deploy']();
 const deploy=f.calls.find(r=>r[0]==='job');assert.equal(deploy[1],'/api/connections/deploy');assert.equal(deploy[2].confirmed,true);
 assert.equal(deploy[2].role,'nos');assert.equal(f.nodes['#setup-password'].value,'');
});
test('an already installed service uses connection rather than reinstalling',async()=>{
 const f=fixture(true);await f.c.setupWizard('nos');assert.equal(f.dialogs.length,0);
 assert.ok(f.calls.some(r=>r[1]==='/api/connections/target'&&r[2].action==='start'));assert.equal(f.calls.some(r=>r[1]==='/api/connections/deploy'),false);
});
test('connection changes invalidate a credential form before it can submit',async()=>{
 const f=fixture();await f.c.setupWizard('nos');f.c.S.connection_generation++;
 await assert.rejects(f.c.ui['setup-deploy'](),/连接设置已变化/);assert.equal(f.calls.some(r=>r[0]==='job'),false);
});
test('explicit service update keeps the one-form install action',async()=>{
 const f=fixture(true);await f.c.setupWizard('nos',{reinstall:true});assert.equal(f.dialogs.length,1);assert.match(f.dialogs[0].title,/更新/);
 await f.c.ui['setup-deploy']();assert.ok(f.calls.some(r=>r[1]==='/api/connections/deploy'));
});
test('automatic relay recovery on the same device keeps its update form valid',async()=>{
 const f=fixture(true);f.c.connections.targets.gos.via_aos=false;
 await f.c.setupWizard('gos',{reinstall:true});f.c.connections.targets.gos.via_aos=true;
 await f.c.ui['setup-deploy']();assert.ok(f.calls.some(r=>r[1]==='/api/connections/deploy'&&r[2].role==='gos'));
});
test('a submitted service update keeps reporting status after the credentials dialog closes',async()=>{
 const f=fixture(true);let finish,isCurrent;
 f.c.job=async(path,body,label,current)=>{isCurrent=current;return new Promise(resolve=>finish=resolve);};
 await f.c.setupWizard('gos',{reinstall:true});
 f.nodes['#setup-admin-password'].value='synthetic';
 const updating=f.c.ui['setup-deploy']();await tick();
 f.nodes['#modal'].close();
 assert.equal(isCurrent(),true);
 f.c.S.connection_generation++;
 assert.equal(isCurrent(),false);
 f.c.S.connection_generation--;
 finish({ok:true});await updating;
 assert.equal(f.nodes['#setup-admin-password'].value,'');
 assert.ok(f.calls.some(r=>r[0]==='toast'&&r[1]==='连接服务已就绪'));
});
test('discovery only runs on click, ignores duplicate clicks and never connects or installs',async()=>{
 const f=fixture();assert.equal(f.calls.length,0);let done;
 f.c.job=async(...args)=>{f.calls.push(['job',...args]);return new Promise(resolve=>done=resolve);};
 const work=f.c.ui['discover-robot']();await f.c.ui['discover-robot']();assert.equal(f.calls.filter(r=>r[0]==='job').length,1);
 done({ok:true,found:true,connections:{...f.c.connections,aos_ip:'found-address'}});await work;
 assert.equal(f.c.connections.aos_ip,'found-address');assert.equal(f.calls.some(r=>r[1]==='/api/connections/connect'||r[1]==='/api/connections/deploy'),false);
});
test('re-pairing requires an explicit confirmation bound to current device settings',async()=>{
 const f=fixture();f.c.repairDevicePairing('nos');assert.equal(f.calls.length,0);assert.match(f.dialogs[0].body,/重新保存.*设备身份/);
 await f.c.ui['confirm-device-pairing']();const call=f.calls.find(r=>r[0]==='job');assert.equal(call[1],'/api/connections/pair');assert.equal(call[2].replace,true);
 const stale=fixture();stale.c.repairDevicePairing('nos');stale.c.connections.targets.nos.host='another';
 await assert.rejects(stale.c.ui['confirm-device-pairing'](),/连接设置/);assert.equal(stale.calls.some(r=>r[0]==='job'),false);
});
test('ordinary connection view keeps the complete profile form collapsed and exposes discovery',()=>{
 const f=fixture();const children=[],form={},content={replaceChildren(...items){children.length=0;children.push(...items);},append(item){children.push(item);}};
 f.nodes['#connection-content']=content;f.nodes['#connections-profile-form']=form;f.nodes['#quick-aos-ip'].addEventListener=()=>{};
 f.c.paintQuickConnections=()=>{};f.c.connectionCenter=()=>{};
 f.c.document.createElement=()=>({innerHTML:'',children:[],append(item){this.children.push(item);}});
 const workflows=fs.readFileSync('platform_web/workflows.js','utf8');vm.runInContext(workflows.slice(workflows.indexOf('const fullConnectionCenter='),workflows.indexOf('function paintQuickConnections()')),f.c);
 f.c.connectionCenter();assert.match(children[0].innerHTML,/discover-robot/);assert.match(children[0].innerHTML,/连接机器人/);assert.doesNotMatch(children[0].innerHTML,/SSH|指纹|ROS|sudo|重新部署/);
 const advanced=children.find(v=>v.id==='connection-advanced');assert.ok(advanced);assert.equal(advanced.open,undefined);assert.ok(advanced.children.includes(form));assert.equal(f.calls.length,0);
});
