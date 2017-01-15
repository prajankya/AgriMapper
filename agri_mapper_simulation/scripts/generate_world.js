var rad = 0.1;
var len = 1;

var ar = 5; //col width/2 i.e. we need 10 so here 5
var vertical_increment = 2;
var horizontal_increment = 2;

//------------------------------------------------------------------------
const path = require('path');
const fs = require('fs');

var top = "<sdf version='1.5'><world name='default'><light name='sun' type='directional'><cast_shadows>1</cast_shadows><pose>0 0 10 0 -0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>" +
  "<attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation><direction>-0.5 0.1 -0.9</direction></light><model name='ground_plane'><static>1</static>" +
  "<link name='link'><collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction><contact><ode/></contact>" +
  "<bounce/></surface><max_contacts>10</max_contacts></collision><visual name='visual'><cast_shadows>0</cast_shadows>" +
  "<geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>" +
  "<self_collide>0</self_collide><kinematic>0</kinematic><gravity>1</gravity></link></model><physics name='default_physics' default='0' type='ode'><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>" +
  "<real_time_update_rate>1000</real_time_update_rate><gravity>0 0 -9.8</gravity><magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field></physics><scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background>" +
  "<shadows>1</shadows></scene><spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>";

function cy1(x, y, z, rad, len) {
  return "<model name='unit_cylinder_" + x + "_" + y + "'><pose>" + x + " " + y + " " + z + " 0 -0 0</pose><link name='link'><inertial><mass>1</mass><inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia></inertial><collision name='collision'>" +
    "<geometry><cylinder><radius>" + rad + "</radius><length>" + len + "</length></cylinder></geometry><max_contacts>10</max_contacts><surface><contact><ode/></contact><bounce/><friction><ode/></friction></surface></collision>" +
    "<visual name='visual'><geometry><cylinder><radius>" + rad + "</radius><length>" + len + "</length></cylinder></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>" +
    "<self_collide>0</self_collide><kinematic>0</kinematic><gravity>1</gravity></link></model>";
}

var mid = "<state world_name='default'><sim_time>91 90000000</sim_time><real_time>91 232076882</real_time><wall_time>1484444207 132898518</wall_time><model name='ground_plane'><pose>0 0 0 0 -0 0</pose><link name='link'><pose>0 0 0 0 -0 0</pose>" +
  "<velocity>0 0 0 0 -0 0</velocity><acceleration>0 0 0 0 -0 0</acceleration><wrench>0 0 0 0 -0 0</wrench></link></model>";

function cy2(x, y, z) {
  return "<model name='unit_cylinder_" + x + "_" + y + "'><pose>" + x + " " + y + " " + z + " 0 -0 1.7e-05</pose><link name='link'><pose>" + x + " " + y + " " + z + " 0 -0 1.7e-05</pose><velocity>0 0 0 0 -0 0</velocity>" +
    "<acceleration>0 0 -9.8 1.76784e-16 -0.891801 0.889857</acceleration><wrench>0 0 -9.8 0 -0 0</wrench></link></model>";
}
var bottom = "</state><gui fullscreen='0'><camera name='user_camera'><pose>5 -5 2 0 0.275643 2.35619</pose><view_controller>orbit</view_controller></camera></gui></world></sdf>";

var out = "";

out += top;

for (var i = -ar; i < ar; i++) {
  for (var j = -ar; j < ar; j++) {
    out += cy1(i * vertical_increment, j * horizontal_increment, 0, rad, len);
  }
}

out += mid;

for (var i = -ar; i < ar; i++) {
  for (var j = -ar; j < ar; j++) {
    out += cy2(i * vertical_increment, j * horizontal_increment, 0);
  }
}
out += bottom;
fs.writeFileSync(path.join("..", "worlds", "compiled.world"), out);
console.log("Done!");