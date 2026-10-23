(() => {
  'use strict';
  const repo='https://github.com/suibianba12138/6-RSS-Vision-Guided-Parallel-Platform/blob/main/%E8%B0%83%E7%94%A8%E5%8F%8D%E8%A7%A3DLL%E5%BA%93/';
  const modes={
    laser:{tab:'laserTab',class:'laser-mode',overlay:'LASER TARGET',output:'Target location + depth',steps:[['HSV + contour detection','Red / white regions'],['Pixel offset + depth','Camera-relative estimate'],['Yaw / pitch setpoint','Experimental proportional logic']],files:'Source modules: vision2.py + shangweiji11.py',source:'vision2.py',caveat:'The visualization describes modules present in code. It is not camera footage, a running simulation or proof of calibrated closed-loop accuracy.'},
    rectangle:{tab:'rectangleTab',class:'rectangle-mode',overlay:'BLACK RECTANGLE',output:'Rectangle geometry + depth',steps:[['Depth + contour processing','Black region / rectangle'],['Shape dimensions + offsets','Estimated target geometry'],['Square-path waypoints','Experimental path generator']],files:'Source modules: visonblack.py + shangweijiblack4.py',source:'shangweijiblack4.py',caveat:'The square-path program generates waypoints and actuator commands in code. Physical square tracking accuracy has not been independently benchmarked.'}
  };
  const $=id=>document.getElementById(id);
  const tabs=Array.from(document.querySelectorAll('.mode-tab'));
  function setMode(mode,focus=false){
    const data=modes[mode];if(!data)return;
    tabs.forEach(tab=>{const active=tab.dataset.mode===mode;tab.classList.toggle('active',active);tab.setAttribute('aria-selected',String(active));tab.tabIndex=active?0:-1;if(active&&focus)tab.focus()});
    $('modePanel').setAttribute('aria-labelledby',data.tab);
    $('visionCanvas').classList.remove('laser-mode','rectangle-mode');$('visionCanvas').classList.add(data.class);
    $('overlayMode').textContent=data.overlay;$('visionOutput').textContent=data.output;
    data.steps.forEach((step,i)=>{$('step'+(i+1)).textContent=step[0];$('detail'+(i+1)).textContent=step[1]});
    $('modeNote').textContent=data.files;$('modeSource').href=repo+data.source;$('modeCaveat').textContent=data.caveat;
  }
  tabs.forEach((tab,i)=>{tab.addEventListener('click',()=>setMode(tab.dataset.mode));tab.addEventListener('keydown',e=>{const keys=['ArrowLeft','ArrowRight','Home','End'];if(!keys.includes(e.key))return;e.preventDefault();const index=e.key==='Home'?0:e.key==='End'?tabs.length-1:(i+(e.key==='ArrowRight'?1:-1)+tabs.length)%tabs.length;setMode(tabs[index].dataset.mode,true)})});
  const menu=$('menuBtn'),nav=$('mainNav');menu?.addEventListener('click',()=>{const open=menu.getAttribute('aria-expanded')!=='true';menu.setAttribute('aria-expanded',String(open));menu.setAttribute('aria-label',open?'Close navigation':'Open navigation');nav.classList.toggle('is-open',open)});
  nav?.querySelectorAll('a').forEach(link=>link.addEventListener('click',()=>{menu.setAttribute('aria-expanded','false');menu.setAttribute('aria-label','Open navigation');nav.classList.remove('is-open')}));
})();
