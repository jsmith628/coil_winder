
part = 0;

M = 1000;
epsilon = 0.01;

tolerance = 0.6;

wall_thickness = 3.0;
shaft_wall_thickness = 0.75;

square_head = false;
rod_diameter = 6.4+0.5;
spool_diameter = rod_diameter + 2*shaft_wall_thickness;
spool_length = 20;

holder_width = 20;
mount_height = 10;
feet_width = 15;
holder_height = 20;
mount_bolt_diameter = 2;

valve_head_thickness = 5;
valve_head_diameter = 17;
valve_fit_diameter = 6;
valve_fit_wall_thickness = 1;

$fn = 100;

module mirror_x(){
    children();
    scale([1,-1]) children();
}

if(part==0 && square_head) {
    translate([0,0,spool_diameter/2])
    rotate([0,90,0])
    difference(){
        union() {
            cylinder(d = spool_diameter, h = spool_length, center=true, $fn=$fn);

            translate([0,0,-wall_thickness/2-spool_length/2+epsilon])
            cube([spool_diameter, holder_width,wall_thickness],center=true);

            translate([0,0,-wall_thickness/2+spool_length/2-epsilon])
            cube([spool_diameter, holder_width,wall_thickness],center=true);
        }
        cylinder(d = rod_diameter, h = M, center=true, $fn=$fn);
    }
}


if(part==0 && !square_head){
    difference() {
        union() {

            linear_extrude(height=holder_height) {
    //            translate([spool_length/2-epsilon,-holder_width/2-wall_thickness])
    //            square([wall_thickness, holder_width+wall_thickness*2]);

                translate([-spool_length/2-wall_thickness+epsilon,-holder_width/2])
                square([wall_thickness, holder_width]);

                mirror_x(){
                    translate([-spool_length/2-mount_height-wall_thickness+epsilon,holder_width/2])
                    square([wall_thickness, feet_width]);

                    translate([-spool_length/2-mount_height-wall_thickness+epsilon,holder_width/2])
                    square([mount_height+wall_thickness, wall_thickness]);

                }
            }

            translate([0,0,holder_height/2])
            rotate([0,90,0])
            cylinder(d=spool_diameter, h=spool_length+epsilon, center=true);

            N = 100;

            translate([spool_length/2,0,holder_height/2])
            rotate([0,90,0])
            rotate([0,0,180/N])
            cylinder(d=holder_width/cos(180/N), h=wall_thickness, $fn=N);

        }

        translate([0,0,holder_height/2])
        rotate([0,90,0])
        cylinder(d=rod_diameter, h=M, center=true);

        mirror_x(){

            translate([0,holder_width/2+feet_width/2,holder_height/2])
            rotate([0,90,0])
            cylinder(d=mount_bolt_diameter+tolerance,h=M,center=true);

        }
    }
}

spool_mount_bolt_diameter = 4;
spool_mount_countersink_diameter = 7.3;
spool_mount_countersink = 2.5;
spool_mount_bolt_spacing = 21;
spool_mount_thickness = wall_thickness+spool_mount_countersink+wall_thickness;

post_length = wall_thickness*1.5;

module mount_bolt_holes() {
    for(i=[0:3]) {
        rotate([0,0,i*90+45])
        translate([(spool_mount_bolt_spacing-spool_mount_bolt_diameter)/sqrt(2),0,0]) {
            cylinder(d=spool_mount_bolt_diameter+tolerance,h=M,center=true);
            translate([0,0,spool_mount_thickness-spool_mount_countersink-wall_thickness])
                cylinder(d=spool_mount_countersink_diameter+tolerance,h=M);
        }
    }
}

if(part==1) {
  difference(){
      cylinder(d=holder_width+feet_width*2, h=spool_mount_thickness);

      mount_bolt_holes();

      translate([0,0,spool_mount_thickness-wall_thickness/2+epsilon])
      cube([holder_height,holder_width+feet_width*2+tolerance*2, wall_thickness],center=true);

  }

  mirror_x(){
      translate([0,holder_width/2+feet_width/2])
      cylinder(d=mount_bolt_diameter,h=post_length+spool_mount_thickness-wall_thickness);
  }
}

if(part == 2) {
  difference(){
      cylinder(d=max(holder_width+wall_thickness*2,spool_mount_bolt_spacing/cos(45)+wall_thickness*2), h=spool_mount_thickness);

      mount_bolt_holes();

      translate([0,0,spool_mount_thickness-wall_thickness+epsilon])
      cylinder(d=holder_width+tolerance,h=wall_thickness);

  }
}

//translate([-spool_length-mount_height/2,0,holder_height/2])
//rotate([0,90,0])
if(part==3) {
    difference() {
        cylinder(d=valve_head_diameter,h=valve_head_thickness);
        translate([0,0,valve_fit_wall_thickness]) cylinder(d=valve_fit_diameter,h=M);
    }
}
