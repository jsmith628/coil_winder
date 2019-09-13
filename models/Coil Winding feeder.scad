
part = 0;

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

/* head_shape = 0;
head_bottom_height = 20;
head_bottom_width = 40;
head_bottom_depth = 60;
head_bottom_length = 110;
head_offset = -10;
head_height = 30;
head_width = 20;
head_length = 150; */

head_shape = 1;
head_bottom_height = 8;
head_bottom_width = 38;
head_bottom_length = 110;
head_offset = -7;
head_bottom_depth = neck_depth-head_offset;
head_height = 24;
head_width = 20;
head_length = 100;

bolt_diameter = 3;
bolt_head_diameter = 5;
bolt_wall_thickness = 2;
bolt_offset = 2;
bolt_length = 20;
nut_size = 6.03;
nut_thickness = 2.4;

guide_diameter = 2;
guide_length = 10;
guide_offset = -50;
guide_wall_thickness = 2.5;
feed_rounding_radius = 0.7;

holder_base_width = 30;
holder_width = 7;
holder_diameter = 10;
holder_offset = 40;
holder_outer_diameter = 15;

do_holder = true;
do_wheels = false;

do_carriage = part==0 || part==1;

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
  [-base_depth/2+neck_offset+wheel_width, neck_height+head_height/2],
  /* [
    -base_depth/2+neck_offset+wheel_width/3,
    neck_height+head_bottom_height - (head_height-head_bottom_height)/2
  ], */
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
        translate([-neck_width/2,-base_depth/2+neck_offset,-epsilon/2])
          cube([neck_width, neck_depth, epsilon]);

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
    } else if(head_shape == 1) {
      hull() {
        translate([-neck_width/2,-base_depth/2+neck_offset,-epsilon/2])
          cube([neck_width, neck_depth, epsilon]);

        translate([-neck_width/2,-base_depth/2+neck_offset,head_height])
          cube([neck_width, neck_depth, epsilon]);

        head_middle_height = head_height-2*head_bottom_height;

        translate([
          -head_bottom_width/2,
          -base_depth/2+neck_offset+head_offset,
          head_bottom_height
        ])
          cube([head_bottom_width, head_bottom_depth, head_middle_height]);

        translate([
          -head_width/2,
          -base_depth/2+neck_offset+head_length,
          head_bottom_height
        ])
          cube([head_width,epsilon,guide_diameter+guide_wall_thickness*2]);

      }
    }
  }


  //the screw holes for the top part
  mirror_x()
  translate([
    -head_bottom_width/2+bolt_diameter/2+bolt_wall_thickness,
    neck_offset-base_depth/2+neck_depth+bolt_offset,
    neck_height+head_height/2
  ])
  rotate([-90,0,0])
  rotate([0,0,180]) {
    screw_hole(bolt_diameter, bolt_length, 3, nut_thickness, bolt_length-nut_thickness*2, M);
    translate([0,0,-epsilon]) cylinder(d=bolt_head_diameter, h=M);
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

  translate([-M/2, -base_depth/2+neck_offset+neck_depth, -M/2])
  if(part==1) {
    cube([M,M,M]);
  } else if(part==2) {
    scale([1,-1,1]) cube([M,M,M]);
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
      rotate([0,0,45])
      cube([guide_diameter+r*2,guide_diameter+r*2,M], center=true);
    }

    sphere(r=feed_rounding_radius);

  }
}

translate([0, -base_depth/2, base_height]) feed_guide();

//the spool holder
if(do_holder) {
  neck_base_width = (base_width-wheel_width)/2;
  real_neck_width = (neck_width-wheel_width)/2;

  angle = atan(neck_height/neck_offset);
  tangent = sqrt(neck_offset*neck_offset + neck_height*neck_height) / (neck_base_width - real_neck_width);

  top_offset = -holder_base_width/tangent;
  top_length = neck_base_width + top_offset;

  echo(tangent);

  translate([0,-base_depth/2, base_height])
  rotate([angle,0,0])
  translate([0,0,-epsilon/2]) {
    //the base
    intersection() {

      //makes the base line up with the outer edge
      mirror_x()
      translate([wheel_width/2,0,0])
      linear_extrude(M)
      polygon(
        [[neck_base_width-holder_width,0],[neck_base_width,0],
        [top_length,holder_base_width], [neck_base_width-holder_width,holder_base_width]]
      );

      union() {
        translate([0,holder_base_width/2,holder_offset])
        rotate([0,-90,0])
        cylinder(d=holder_outer_diameter,h=M,center=false);

        //the general shape
        translate([wheel_width/2,0,0])
        hull() {

          translate([0,holder_base_width/2,holder_offset])
          rotate([0,90,0])
          cylinder(d=holder_outer_diameter,h=M,center=false);

          cube([M,holder_base_width,epsilon]);

        }
      }


    }

    //the actual holder
    translate([0,holder_base_width/2,holder_offset])
    rotate([0,-90,0])
    cylinder(d=holder_diameter,h=base_width-holder_width*2,,center=true);

  }


}


if(part==2 || part==0) {
  if(head_shape == 0) {
    translate(
      [0,
      neck_offset-base_depth/2+head_length + guide_offset - wheel_width,
      base_height+neck_height+head_bottom_height + (head_height-head_bottom_height)/2-wheel_width/2]
    )
      feed_guide();
  } else if(head_shape == 1) {
    translate(
      [0,
      neck_offset-base_depth/2+head_length + head_bottom_depth + guide_offset - wheel_width,
      base_height+neck_height+head_bottom_height+feed_rounding_radius]
    )
      feed_guide();
  }
}
