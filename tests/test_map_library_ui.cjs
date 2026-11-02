const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');const vm=require('node:vm');
function fixture(){
 let writes=0,markup='';const list={set innerHTML(v){writes++;markup=v;},get innerHTML(){return markup;}};
 const nodes={'#remote-map-list':list,'[data-ui="map-library-edit"]':{},'[data-ui="map-library-trash"]':{}};
 const dialogs=[];
 const c=vm.createContext({S:{mode:'demo',ip:null},page:'maps',mapLibrary:{management_version:1,desktop_management_version:1,active:'base',maps:[{name:'candidate',display_name:'走廊更新',cleanup_base:'base',modified:3,grids:[0]},{name:'base',display_name:'一楼',active:true,modified:1,grids:[0]}]},selectedRemote:'base',ui:{},paintMapLibrary(){},$:s=>nodes[s],text(){},esc:s=>String(s),icon:()=>'',sizeLabel:()=>'',empty:t=>t,document:{addEventListener(){}},modal:(title,body)=>dialogs.push({title,body})});
 vm.runInContext(fs.readFileSync('platform_web/map_library.js','utf8'),c);
 return {c,dialogs,list,get writes(){return writes;}};
}
test('unchanged polling preserves editor DOM and button focus',()=>{
 const f=fixture();f.c.paintMapLibrary();f.c.ui['map-library-edit']();const count=f.writes;
 f.c.paintMapLibrary();f.c.paintMapLibrary();assert.equal(f.writes,count);
 assert.match(f.list.innerHTML,/data-map-delete="base" disabled/);
});
test('active map comes first and search matches labels or stable identities',()=>{
 const f=fixture();assert.equal(f.c.visibleLibraryMaps()[0].name,'base');
 vm.runInContext("mapSearch='走廊'",f.c);assert.deepEqual(Array.from(f.c.visibleLibraryMaps(),m=>m.name),['candidate']);
 vm.runInContext("mapSearch='base'",f.c);assert.deepEqual(Array.from(f.c.visibleLibraryMaps(),m=>m.name),['base']);
});
test('automatic cleanup is offered only for a complete candidate of the active base',()=>{
 const f=fixture();f.c.openMapActivation('candidate',0);assert.match(f.dialogs.at(-1).body,/map-cleanup-previous/);
 f.c.mapLibrary.active='unrelated';f.c.openMapActivation('candidate',0);assert.doesNotMatch(f.dialogs.at(-1).body,/map-cleanup-previous/);
});
test('old NOS capability does not offer a nonworking editor',()=>{
 const f=fixture();delete f.c.mapLibrary.management_version;f.c.paintMapLibrary();
 f.c.ui['map-library-edit']();assert.doesNotMatch(f.list.innerHTML,/data-map-delete/);
});
