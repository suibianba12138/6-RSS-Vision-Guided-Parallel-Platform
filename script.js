(() => {
  "use strict";
  const base = "https://github.com/suibianba12138/6-RSS-Vision-Guided-Parallel-Platform/blob/main/%E8%B0%83%E7%94%A8%E5%8F%8D%E8%A7%A3DLL%E5%BA%93/";
  const modes = {
    laser: {
      overline: "VISION ROUTE / A",
      title: "Tracking a visual target",
      description: "A red/white feature detector uses color and depth measurements to estimate a laser target's position. An experimental upper-computer controller converts offsets into orientation commands.",
      files: ["vision2.py", "shangweiji11.py"],
      stages: [
        ["INPUT", "RGB + depth", "RealSense stream"],
        ["PERCEPTION", "Target features", "HSV / contours"],
        ["CONTROL", "Pose estimate", "Offset → yaw/pitch"],
        ["OUTPUT", "Six servo angles", "DLL → serial → ESP32"]
      ],
      caveat: "This represents source-code intent. Camera-to-platform calibration and stable closed-loop tracking have not been independently validated."
    },
    square: {
      overline: "VISION ROUTE / B",
      title: "Following a geometric path",
      description: "A black-rectangle detector estimates the target's dimensions and location. A separate desktop controller builds a square waypoint sequence and maps it into experimental actuator commands.",
      files: ["visonblack.py", "shangweijiblack4.py"],
      stages: [
        ["INPUT", "RGB + depth", "RealSense stream"],
        ["PERCEPTION", "Rectangle", "Depth / contours"],
        ["PLANNING", "Square waypoints", "Four edges"],
        ["OUTPUT", "Six servo angles", "DLL → serial → ESP32"]
      ],
      caveat: "The generated waypoints and inverse-kinematics calls are source-visible. Accurate square motion on real hardware is not independently demonstrated or benchmarked."
    }
  };
  const $ = (id) => document.getElementById(id);
  const tabs = Array.from(document.querySelectorAll(".route-tab"));
  function selectMode(name, focus = false) {
    const data = modes[name];
    if (!data) return;
    tabs.forEach(tab => {
      const active = tab.dataset.mode === name;
      tab.classList.toggle("is-active", active);
      tab.setAttribute("aria-selected", String(active));
      tab.tabIndex = active ? 0 : -1;
      if (active && focus) tab.focus();
    });
    $("routeContent").setAttribute("aria-labelledby", "tab-" + name);
    $("routeOverline").textContent = data.overline;
    $("routeTitle").textContent = data.title;
    $("routeDescription").textContent = data.description;
    $("routeCaveat").textContent = data.caveat;
    const links = $("routeFiles");
    links.replaceChildren();
    const label = document.createElement("span");
    label.textContent = "KEY MODULES";
    links.append(label);
    data.files.forEach(file => {
      const link = document.createElement("a");
      link.href = base + encodeURIComponent(file);
      link.textContent = file + " ↗";
      link.target = "_blank";
      link.rel = "noopener noreferrer";
      links.append(link);
    });
    const steps = $("routeSteps");
    steps.replaceChildren();
    steps.setAttribute("aria-label", name === "laser" ? "Laser target processing stages" : "Rectangle trajectory processing stages");
    for (const [labelText, title, caption] of data.stages) {
      const step = document.createElement("div"); step.className = "step";
      const tag = document.createElement("small"); tag.textContent = labelText;
      const heading = document.createElement("strong"); heading.textContent = title;
      const desc = document.createElement("span"); desc.textContent = caption;
      step.append(tag, heading, desc);
      steps.append(step);
    }
  }
  tabs.forEach(tab => {
    tab.addEventListener("click", () => selectMode(tab.dataset.mode));
    tab.addEventListener("keydown", event => {
      if (event.key !== "ArrowLeft" && event.key !== "ArrowRight" && event.key !== "Home" && event.key !== "End") return;
      event.preventDefault();
      const index = tabs.indexOf(tab);
      const target = event.key === "Home" ? 0 : event.key === "End" ? tabs.length-1 : (index + (event.key === "ArrowRight" ? 1 : -1) + tabs.length) % tabs.length;
      selectMode(tabs[target].dataset.mode, true);
    });
  });
  const menu = $("menuButton"), nav = $("primaryNav");
  if (menu && nav) {
    menu.addEventListener("click", () => {
      const open = menu.getAttribute("aria-expanded") !== "true";
      menu.setAttribute("aria-expanded", String(open));
      nav.classList.toggle("is-open", open);
    });
    nav.querySelectorAll("a").forEach(link => link.addEventListener("click", () => {
      menu.setAttribute("aria-expanded", "false");
      nav.classList.remove("is-open");
    }));
  }
  const progress = $("readingProgress");
  let pending = false;
  function updateScroll() {
    pending = false;
    const available = document.documentElement.scrollHeight - window.innerHeight;
    if(progress) progress.style.width = (available > 0 ? Math.max(0, Math.min(100, window.scrollY / available * 100)) : 0) + "%";
  }
  window.addEventListener("scroll", () => {
    if (!pending) { pending = true; requestAnimationFrame(updateScroll); }
  }, {passive:true});
  updateScroll();
  selectMode("laser");
})();