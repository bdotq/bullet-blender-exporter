Before exporting the physical scene, you should

- rotate object in [edit mode](https://docs.blender.org/manual/en/dev/modeling/meshes/introduction.html#edit-mode) to align the [bound box](https://docs.blender.org/manual/en/dev/editors/3dview/properties/shading.html#viewport-shading)
- [set the origin](https://docs.blender.org/manual/en/dev/editors/3dview/object/origin.html#set-origin) of geometry to it's [bounds center](https://docs.blender.org/manual/en/dev/editors/3dview/object/editing/transform/control/pivot_point/bounding_box_center.html?highlight=bounding%20box)
- [apply scale](https://docs.blender.org/manual/en/dev/editors/3dview/object/editing/transform/clear_apply.html?highlight=apply%20object%20transformations#apply) to object data

Update Dec-2020
- Added option to Export only selected objects

Install
- In the Blender program folder, navigate to [[2.8x\scripts\addons\]]
- Create a folder in 'addons' and name it something like "io_bullet_blender_exporter" (no spaces, hyphens)
- Copy all the files in this project to the folder
- Start Blender, open Preferences > Add-ons enable "Bullet json format" 
