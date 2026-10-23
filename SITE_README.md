# 6-RSS Showcase Website

Redesigned V2 static, responsive portfolio site for the [6-RSS Vision-Guided Parallel Platform](https://github.com/suibianba12138/6-RSS-Vision-Guided-Parallel-Platform).

The site is intended for GitHub Pages from the `gh-pages` branch at repository root.

## Website V2

The redesigned presentation uses a spacious typographic hero, conceptual robot illustration, a dark interactive walkthrough, architecture breakdown, and an explicit pending real-hardware video panel. The interaction explains code; it is not a live experiment.

## Preview locally

From this branch's root, run `python3 -m http.server 8000`, then open `http://localhost:8000/`. No package installation, build tool or API is needed.

## Evidence statement

The robot in the hero section is an **illustrative SVG schematic**, not a photo, CAD-derived exact model, measured mechanism geometry or simulation result. The interactive vision explorer contains two selectable source-documented routes (laser target and black rectangle). Both pathways are diagrams representing separate modules found in the source repository; tab selection does **not** execute a control routine or perform live perception. The video panel is deliberately marked **coming soon**, and no performance metrics have been invented.

## Publishing on GitHub Pages

Repository **Settings → Pages → Build and deployment → Deploy from a branch → Branch: `gh-pages`, folder: `/(root)` → Save**. GitHub must have Pages enabled for the site to appear. A published URL is expected to be `https://suibianba12138.github.io/6-RSS-Vision-Guided-Parallel-Platform/`; this is not evidence of a successful deployment until Pages reports a build and the URL is reachable.

## Adding the real video later

Keep the original footage separate from the illustrative schematic. Review media privacy, people and licenses. Upload a small approved MP4/GIF under a clearly labeled media path or host it through an approved Pages/media mechanism. Replace only the video placeholder after provenance is documented. Do not imply a quantitative accuracy result based on a visual demonstration alone.

## Repository layout

`index.html` — page structure and accessibility semantics  
`style.css` — responsive layouts and styles  
`script.js` — interactive vision-route tabs and mobile navigation  
`favicon.svg` — branding  
`docs/assets/system-overview.svg` — shared source-grounded architecture figure from the main branch

No original controller code was modified to build this showcase.
