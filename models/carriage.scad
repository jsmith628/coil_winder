

M=1000;
epsilon = 0.01;

smooth_rod_diameter = 8;
threaded_rod_diameter = 8;
threaded_rod_tolerance = 0.25;

linear_bearing_diameter = 15.2;
linear_bearing_length = 23.8;

threaded_bearing_length = 13.5;
threaded_bearing_diameter = 10.16 + 0.8;
threaded_bearing_mount_diameter = 22.5 + 0.8;
threaded_bearing_mount_countersink = 3.5;
threaded_bearing_bolt_spacing = 11;

threaded_bearing_bolt_size = 3;
threaded_bearing_bolt_length = 30;
threaded_bearing_nut_size = 6.03;
threaded_bearing_nut_thickness = 2.4;

rod_spacing = 35;
base_height = 30;
base_width = 48;
base_depth = 100;

flange_bearing_max_diameter = 27.0;
flange_bearing_max_width = 47.5;
flange_bearing_thickness = 4.05;
flange_bearing_bolt_diameter = 5;
flange_bearing_bolt_offset = 3.0;
flange_bearing_min_diameter = flange_bearing_bolt_diameter + 2*flange_bearing_bolt_offset;

do_carriage = true;

module flange_bearing() {

    hull() {

        circle(d=flange_bearing_max_diameter);

        mirror_x()
            translate([-flange_bearing_max_width/2+flange_bearing_min_diameter/2,0,0])
            circle(d=flange_bearing_min_diameter);

    }
}

module mirror_x() {
    children();
    scale([-1,1,1]) children();
}

module mirror_y() {
    children();
    scale([1,-1,1]) children();
}

module screw_hole(Diameter, Length, NutSize, NutThickness, NutDepth, SlotLength) {
    scale([1,1,-1])
        cylinder(d=Diameter, h=Length, center=false);

    translate([0,0,-NutDepth])
        rotate([180,0,0])
        linear_extrude(center=SlotLength>0, height=NutThickness)
        if(SlotLength == 0){
            polygon([
                [NutSize/2*tan(30),NutSize/2],
                [-NutSize/2*tan(30),NutSize/2],
                [-NutSize/2/cos(30),0],
                [-NutSize/2*tan(30),-NutSize/2],
                [NutSize/2*tan(30),-NutSize/2],
                [NutSize/2/cos(30),0]
            ]);
        } else {
            polygon([
                [SlotLength,NutSize/2],
                [-NutSize/2*tan(30),NutSize/2],
                [-NutSize/2/cos(30),0],
                [-NutSize/2*tan(30),-NutSize/2],
                [SlotLength,-NutSize/2]
                //[NutSize/2/cos(30),0],
            ]);
        }

}


//
//The base
//

/*$fa = 1;
$fs = 0.2;

flange_bearing();*/

if(do_carriage)
translate([0,0,base_height/2])
difference() {
    cube([base_width, base_depth, base_height], center=true);

    //The hole for the threaded rod
    rotate([0,90,0])
        cylinder(h=M, d=threaded_rod_diameter+2*threaded_rod_tolerance, center=true);

    //the threaded rod mount
    translate([base_width/2-threaded_bearing_length,0,0])
    rotate([0,90,0])
        cylinder(h=M, d=threaded_bearing_diameter);

    //the threaded rod mount countersink
    translate([base_width/2-threaded_bearing_mount_countersink,0,0])
        rotate([0,90,0])
        cylinder(h=M, d=threaded_bearing_mount_diameter);

    //the mounting screw-holes
    translate([base_width/2,0,0])for(i=[0:3]) {
        rotate([90*i,0,0])
        rotate([45,0,0])
        translate([0,threaded_bearing_bolt_spacing/sqrt(2),0])
            rotate([0,90,0])
            rotate([0,0,0])
            screw_hole(
                threaded_bearing_bolt_size,
                threaded_bearing_bolt_length,
                threaded_bearing_nut_size,
                M,
                threaded_bearing_bolt_length-threaded_bearing_nut_thickness,
                0
                //threaded_bearing_mount_diameter/2
            );
    }

    //the holes for the linear bearings
    mirror_y()
        translate([0,-rod_spacing])
        rotate([0,90,0])
        cylinder(h=M, d=linear_bearing_diameter, center=true);

}
