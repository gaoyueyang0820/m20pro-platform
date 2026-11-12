import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
window.createRobotViewer = function(element) {
  const scene = new THREE.Scene(); scene.background = new THREE.Color('#f3f7f8');
  const camera = new THREE.PerspectiveCamera(36, 1, .01, 30); camera.up.set(0,0,1); camera.position.set(1.6,-1.9,1.3);
  const renderer = new THREE.WebGLRenderer({antialias:true,alpha:false}); renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.shadowMap.enabled=true; renderer.shadowMap.type=THREE.PCFSoftShadowMap; renderer.outputColorSpace=THREE.SRGBColorSpace;
  element.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera,renderer.domElement); controls.target.set(0,0,.37); controls.enableDamping=true; controls.minDistance=.7; controls.maxDistance=5; controls.maxPolarAngle=Math.PI*.48;
  scene.add(new THREE.HemisphereLight(0xffffff,0x70828b,2));
  const light=new THREE.DirectionalLight(0xffffff,2.6); light.position.set(2,-3,4); light.castShadow=true; light.shadow.mapSize.set(2048,2048); light.shadow.camera.left=-2; light.shadow.camera.right=2; light.shadow.camera.top=2; light.shadow.camera.bottom=-2; scene.add(light);
  const fill=new THREE.DirectionalLight(0xa7ffe7,1.1);fill.position.set(-2,2,2);scene.add(fill);
  const plane=new THREE.Mesh(new THREE.PlaneGeometry(40,40),new THREE.ShadowMaterial({color:0x648182,opacity:.17}));plane.receiveShadow=true;scene.add(plane);
  const grid=new THREE.GridHelper(8,40,0xb3ced0,0xdde7e8);grid.rotation.x=Math.PI/2;grid.position.z=-.003;scene.add(grid);
  const ring=new THREE.Mesh(new THREE.RingGeometry(.68,.683,90),new THREE.MeshBasicMaterial({color:0x39b89d,side:THREE.DoubleSide,transparent:true,opacity:.45}));ring.position.z=.003;scene.add(ring);
  let robot=null,disposed=false,selected=null,frame,preview=null,legs=[];
  const standingHeight=.58, loweredHeight=.19, wheelRadius=.09;
  let targetHeight=standingHeight,fitLegs=false,lastFrame=performance.now();
  const orientation=new THREE.Quaternion(),bodyEuler=new THREE.Euler(0,0,0,'ZYX');
  const bodyUp=new THREE.Vector3(),wheelOffset=new THREE.Vector3();
  const referenceJoints={fl_hipy_joint:-.3,fl_knee_joint:.6,fr_hipy_joint:-.3,fr_knee_joint:.6,hl_hipy_joint:.3,hl_knee_joint:-.6,hr_hipy_joint:.3,hr_knee_joint:-.6};
  const loader=new URDFLoader();
  loader.manager.onLoad=()=>{if(disposed||!robot)return;robot.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true;let p=o;while(p&&!p.isURDFLink)p=p.parent;const name=p?.name||'';o.userData.link=name;const color=name.includes('wheel')?'#25323c':name==='base_link'?'#304a55':'#8cabb1';o.material=new THREE.MeshStandardMaterial({color,metalness:.35,roughness:.55});o.userData.base=color;}});selected=null;element.dispatchEvent(new CustomEvent('modelready'));};
  loader.load('/assets/model/M20.urdf', r=>{
    if(disposed)return;
    robot=r;
    legs=['fl','fr','hl','hr'].map(name=>{
      const hip=r.joints[name+'_hipx_joint'],knee=r.joints[name+'_knee_joint'],wheel=r.joints[name+'_wheel_joint'];
      const bend=name[0]==='f'?1:-1;
      const offset=hip.position.clone().add(knee.position).add(wheel.position);
      offset.z=hip.position.z;
      return {name,bend,offset,thigh:-knee.position.z,shank:-wheel.position.z,
              minCosine:Math.cos(bend>0?knee.limit.upper:knee.limit.lower)};
    });
    r.position.z=targetHeight;
    scene.add(r);
    r.setJointValues(referenceJoints);
  },undefined,()=>element.dispatchEvent(new CustomEvent('modelerror')));
  const resize=()=>{const w=element.clientWidth,h=element.clientHeight;if(!w||!h)return;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();};const observer=new ResizeObserver(resize);observer.observe(element);resize();
  function updateLegStance(){
    // Approximate support on a flat surface using the measured base and the URDF lengths.
    const vertical=bodyUp.set(0,0,1).applyQuaternion(robot.quaternion).z;
    for(const {name,bend,offset,thigh,shank,minCosine} of legs){
      const offsetZ=wheelOffset.copy(offset).applyQuaternion(robot.quaternion).z;
      const clearance=robot.position.z-wheelRadius;
      const reach=Math.max(0,vertical>.3?(clearance+offsetZ)/vertical:clearance);
      const cosine=(reach*reach-thigh*thigh-shank*shank)/(2*thigh*shank);
      const knee=bend*Math.acos(THREE.MathUtils.clamp(cosine,minCosine,1));
      const hip=-Math.atan2(shank*Math.sin(knee),thigh+shank*Math.cos(knee));
      robot.setJointValue(name+'_hipy_joint',hip);
      robot.setJointValue(name+'_knee_joint',knee);
    }
  }
  function animate(now){
    if(disposed)return;
    frame=requestAnimationFrame(animate);
    const blend=1-Math.exp(-Math.max(0,now-lastFrame)/90);
    lastFrame=now;
    if(robot){
      robot.quaternion.slerp(orientation,blend);
      robot.position.z+=(targetHeight-robot.position.z)*blend;
      if(fitLegs)updateLegStance();
    }
    controls.update();
    renderer.render(scene,camera);
  }
  animate(lastFrame);
  return {update(joints,leg,attitude,posture){
    if(attitude)orientation.setFromEuler(bodyEuler.set(attitude.roll,attitude.pitch,attitude.yaw,'ZYX'));
    else orientation.identity();
    fitLegs=Boolean(attitude||posture)&&!(joints||[]).some(j=>j.fresh&&j.part!=='Wheel'&&Number.isFinite(j.position));
    targetHeight=Number.isFinite(posture?.height)?posture.height:
      posture?.motion_state===4?loweredHeight:standingHeight;
    if(fitLegs)targetHeight=Math.max(loweredHeight,targetHeight);
    if(!robot)return;
    for(const j of joints||[]){
      const position=preview?.[j.index]??(j.fresh?j.position:null);
      robot.setJointValue(j.name,Number.isFinite(position)?position:(referenceJoints[j.name]??0));
    }
    if(selected!==leg){selected=leg;robot.traverse(o=>{if(o.isMesh)o.material.color.set(leg&&o.userData.link?.startsWith(leg.toLowerCase())?'#00b89c':o.userData.base);});}
  },preview(values){preview=values;},reset(){camera.position.set(1.6,-1.9,1.3);controls.target.set(0,0,.37);},dispose(){disposed=true;cancelAnimationFrame(frame);observer.disconnect();controls.dispose();scene.traverse(o=>{if(o.isMesh){o.geometry?.dispose();if(o.material?.dispose)o.material.dispose();}});renderer.dispose();renderer.domElement.remove();}};
};

window.createCloudViewer=function(element){
 const scene=new THREE.Scene();scene.background=new THREE.Color('#142a35');
 const camera=new THREE.PerspectiveCamera(48,1,.01,20000);camera.up.set(0,0,1);camera.position.set(22,-25,23);
 const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));element.appendChild(renderer.domElement);
 const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.maxPolarAngle=Math.PI*.96;
 const geometry=new THREE.BufferGeometry();const material=new THREE.ShaderMaterial({transparent:true,depthWrite:true,uniforms:{size:{value:.065},zmin:{value:-1e7},zmax:{value:1e7},low:{value:0},high:{value:3},mono:{value:0}},vertexShader:`uniform float size;uniform float low;uniform float high;varying float height;varying float z;void main(){z=position.z;height=clamp((position.z-low)/max(high-low,.01),0.,1.);vec4 p=modelViewMatrix*vec4(position,1.);gl_PointSize=clamp(size*700./max(-p.z,.1),1.,12.);gl_Position=projectionMatrix*p;}`,fragmentShader:`uniform float zmin;uniform float zmax;uniform float mono;varying float height;varying float z;void main(){if(z<zmin||z>zmax)discard;vec2 p=gl_PointCoord-.5;if(dot(p,p)>.25)discard;vec3 a=vec3(.18,.77,.64);vec3 b=vec3(.98,.73,.25);gl_FragColor=vec4(mix(a,b,height*(1.-mono)),.94);}`});
 const cloud=new THREE.Points(geometry,material);scene.add(cloud);
 let grid=new THREE.GridHelper(50,50,0x42606a,0x253f4b);grid.rotation.x=Math.PI/2;grid.position.z=-.03;scene.add(grid);scene.add(new THREE.AxesHelper(2));
 const pose=new THREE.Group(),sphere=new THREE.Mesh(new THREE.SphereGeometry(.16,12,12),new THREE.MeshBasicMaterial({color:0xffffff}));pose.add(sphere);const arrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,.2),1,0xffffff,.35,.18);pose.add(arrow);pose.visible=false;scene.add(pose);
 let bounds=null,frame,closed=false;
 const observer=new ResizeObserver(()=>{const w=element.clientWidth,h=element.clientHeight;if(!w||!h)return;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();});observer.observe(element);
 function fit(){if(!bounds)return;const mid=new THREE.Vector3(...bounds[0]).add(new THREE.Vector3(...bounds[1])).multiplyScalar(.5);const span=Math.max(1,...bounds[1].map((v,i)=>v-bounds[0][i]));controls.target.copy(mid);camera.position.copy(mid).add(new THREE.Vector3(span*.65,-span*.8,span*.7));camera.far=Math.max(20000,span*10);camera.updateProjectionMatrix();grid.position.x=mid.x;grid.position.y=mid.y;}
 function animate(){if(closed)return;frame=requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);}animate();
 return {load(data,fitView=true){bounds=data.bounds;geometry.setAttribute('position',new THREE.Float32BufferAttribute(data.points.flat(),3));geometry.computeBoundingSphere();material.uniforms.low.value=bounds[0][2];material.uniforms.high.value=bounds[1][2];if(fitView){material.uniforms.zmin.value=bounds[0][2]-.1;material.uniforms.zmax.value=bounds[1][2]+.1;fit();}},style(v){if(v.size!=null)material.uniforms.size.value=v.size;if(v.zmin!=null)material.uniforms.zmin.value=v.zmin;if(v.zmax!=null)material.uniforms.zmax.value=v.zmax;if(v.mono!=null)material.uniforms.mono.value=v.mono?1:0;if(v.grid!=null)grid.visible=v.grid;if(v.light!=null)scene.background.set(v.light?'#f0f5f4':'#142a35');},pose(p,valid){pose.visible=!!valid;if(valid){pose.position.set(p.PosX,p.PosY,.2);pose.rotation.z=p.Yaw||0;}},reset:fit,dispose(){closed=true;cancelAnimationFrame(frame);observer.disconnect();controls.dispose();geometry.dispose();material.dispose();scene.traverse(o=>{if(o.isMesh){o.geometry?.dispose();o.material?.dispose();}});renderer.dispose();renderer.domElement.remove();}};
};
