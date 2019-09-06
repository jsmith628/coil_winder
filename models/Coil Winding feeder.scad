
include <carriage.scad>

wheel_width = 10.23;
wheel_diameter = 24;
axel_diameter = 5;

neck_height = 80;
neck_depth = 40;
neck_width = 25;
neck_offset = base_depth - neck_depth + 10;
neck_base_offset = 0;
neck_indent_size_factor = [1,1.5];

neck_hole_depth = 5;

head_shape = 0;

head_bottom_height = 20;
head_bottom_width = 40;
head_bottom_depth = 60;
head_bottom_length = 110;
head_offset = -10;
head_height = 30;
head_width = 20;
head_length = 150;

guide_diameter = 2;
guide_length = 10;
guide_offset = -50;
guide_wall_thickness = 2.5;
feed_rounding_radius = 0.7;

do_wheels = false;

$fa = 2;
$fs = 0.2;

/* hole_center = neck_base_offset/2+neck_depth/4; */
hole_center = base_depth/2 - neck_indent_size_factor[0] * neck_depth/2;
neck_center = neck_offset+neck_depth/2 - base_depth/2;


//h1 = wheel_diameter+wheel_width;
//h2 = neck_height+wheel_diameter/3;
//h3 = (h1+h2)/2;

h1 = wheel_diameter;
h3 = neck_height-wheel_diameter/3;
h2 = (h1+h3)/2.5;

slope1 = neck_height/(neck_base_offset-neck_offset);
slope2 = neck_height/(base_depth-(base_depth-neck_base_offset-neck_depth/2)/4 - neck_center-neck_depth/4);
slope3 = neck_height/(hole_center-neck_center);

wheel_positions = [
[neck_center+neck_depth/4+(neck_height-h1)/slope3,h1],
[-base_depth/2 + neck_base_offset+neck_depth/4-h2/slope1,h2],
[neck_center+neck_depth/4+(neck_height-h3)/slope3,h3],
[-base_depth/2+neck_offset+wheel_width/3, neck_height+head_bottom_height -(head_height-head_bottom_height)/2],
];

translate([0,0,base_height])
difference() {

  union() {
    //the neck
    hull() {
      translate([0,0,-epsilon/2])
      linear_extrude(height = epsilon)
      translate([-base_width/2,-base_depth/2+neck_base_offset])
      square([base_width,base_depth-neck_base_offset]);

      translate([0,0,neck_height+epsilon/2])
      linear_extrude(height = epsilon)
      translate([-neck_width/2,-base_depth/2+neck_offset])
      square([neck_width,neck_depth]);
    }

    //the head
    translate([0,0,neck_height])
    if(head_shape == 0) {
      hull() {
        translate([0,0,-epsilon/2])
        linear_extrude(height = epsilon)
        translate([-neck_width/2,-base_depth/2+neck_offset])
        square([neck_width,neck_depth]);

        translate([0,0,head_bottom_height])
        linear_extrude(height = epsilon)
        polygon([
          [head_bottom_width/2, -base_depth/2+neck_offset+head_bottom_depth],
          [head_bottom_width/2, -base_depth/2+neck_offset],
          [-head_bottom_width/2, -base_depth/2+neck_offset],
          [-head_bottom_width/2, -base_depth/2+neck_offset+head_bottom_depth],
          [0, -base_depth/2+neck_offset+head_bottom_length],
          ]);

        translate([0,0,head_height+epsilon/2])
        linear_extrude(height = epsilon)
        polygon([
          [head_width/2, -base_depth/2+neck_offset+head_width],
          [head_width/2, -base_depth/2+neck_offset+head_offset],
          [-head_width/2, -base_depth/2+neck_offset+head_offset],
          [-head_width/2, -base_depth/2+neck_offset+head_width],
          [0, -base_depth/2+neck_offset+head_length],
          ]);
        }
    } else if(head_shape==1) {
      hull() {

      }
    }
  }

  //the slot for the wheel
  translate([-wheel_width/2,-M/2,-epsilon]) cube([wheel_width,M,M]);

  angle = atan((base_width/2 - neck_width/2)/neck_height);

  //the hole in the neck
  mirror_x()
  translate([-base_width/2+epsilon,0,0])
  rotate([0,angle,0])
  translate([neck_hole_depth,0,0])
  rotate([0,-90,0])
  linear_extrude(height=M,center=false)
  polygon([
    [neck_height, neck_center],
    [0, neck_base_offset+neck_indent_size_factor[1] * neck_depth/2-base_depth/2],
    [0, hole_center],
    ]);

    //the holes for the wheels
    for(p = wheel_positions) {
      translate([0,p[0],p[1]])
      rotate([0,90,0])
      cylinder(d=axel_diameter,h=M,center=true);
    }

}

//the wheels (for testing)
if(do_wheels)
for(p = wheel_positions) {
  translate([0,p[0],p[1]+base_height])
  rotate([0,90,0])
  cylinder(d=wheel_diameter,h=wheel_width+epsilon,center=true);
}

module feed_guide() {
  h = guide_wall_thickness*2 + guide_diameter;
  r = feed_rounding_radius;
  minkowski() {
    difference() {
      translate([-wheel_width/2,feed_rounding_radius,0])
      cube([wheel_width,guide_length-r*2,h-r*2]);

      translate([0,0,h/2-r])
      rotate([90,0,0])
      cylinder(d=guide_diameter+r*2,h=M,center=true);
    }

    sphere(r=feed_rounding_radius);

  }
}

translate(
  [0,
  neck_offset-base_depth/2+head_length + guide_offset - wheel_width,
  base_height+neck_height+head_bottom_height + (head_height-head_bottom_height)/2-wheel_width/2]
)
  feed_guide();
