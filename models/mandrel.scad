
M = 100;
epsilon = 0.01;

wall_thickness = 3;

drive_fit_width = 20;
drive_fit_length = 50;

chuck_fit_diameter = 5;
chuck_fit_length = 6;

spring_diameter = 7;
wire_diameter = 0.255;

c0 = 0.980364;
c1 = -0.012436;

dw = wire_diameter/25.4;
ds = dw + spring_diameter/25.4;
fact = c0 + c1*(ds/dw);

diameter = (fact*ds - dw) * 25.4;
length = 50;

wire_hole_diameter = 1;

$fs = 0.1;

difference() {
  //the actual mandrel
  cylinder(d = diameter, h=wall_thickness+length);

  //the slot that the wire fixes into at the end
  translate([0,0,wall_thickness*2])
  rotate([0,90,0])
  cylinder(d=wire_hole_diameter, h=M, center = true);
}

//the rectangular piece that fits onto the drive mount
translate([0,0,wall_thickness/2])
  cube([drive_fit_width, drive_fit_length, wall_thickness], center = true);

//the circular fit that goes into the chuck mount
translate([0,0,wall_thickness+length])
  cylinder(d = chuck_fit_diameter, h=chuck_fit_length);
