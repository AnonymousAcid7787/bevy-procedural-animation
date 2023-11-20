use bevy::prelude::*;
use bevy_rapier3d::{prelude::*, rapier::prelude::{JointLimits, MotorModel}};
use smallvec::SmallVec;

use crate::stickman::{StickmanArmSegment, StickmanArm, ArmMotorParams, SegmentInfo};

macro_rules! spawn_body_part {
    ($mesh:expr, $commands:expr, $material:expr, $transform:expr, $capsule_depth:expr, $radius:expr) => {
        {
            $commands.spawn((
                PbrBundle {
                    mesh: $mesh, 
                    material: $material,
                    transform: $transform,
                    ..Default::default()
                },
                RigidBody::Fixed,
                Collider::capsule(
                    Vec3::Y * $capsule_depth/2.,
                    Vec3::Y * -$capsule_depth/2.,
                    $radius
                ),
            )).id()
        }
    };
}

macro_rules! add_child {
    ($commands:expr, $parent:expr, $child:expr) => {
        $commands.add(AddChild {
            parent: $parent,
            child: $child,
        });
    };
}

pub fn stickman_body_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
) {

    let scale = 1.;

    let radius = 0.03_f32 * scale;
    let torso_depth = 0.6_f32 * scale;
    let torso_len = torso_depth + radius*2.;
    let arm_depth = torso_depth;
    let arm_len = arm_depth + radius*2.;
    let leg_depth = 0.75 * scale;
    let leg_len = leg_depth + radius*2.;
    let latitudes = 8;
    let longitudes = 16;
    let arm_segment_depth = arm_depth/2.;
    let arm_segment_len = arm_segment_depth+radius;

    //motor params
    let target_pos = f32::to_radians(0.);
    let target_vel = f32::to_radians(30.);
    let stiffness = 1.;
    let damping = 0.03;

    let arm_segment = shape::Capsule {
        depth: arm_segment_depth,
        radius,
        latitudes,
        longitudes,
        ..Default::default()
    };

    
    let material = standard_materials.add(Color::PURPLE.into());
    let upper_arm_material = standard_materials.add(Color::ORANGE.into());
    
    //body assembly
    {
        // //shapes
        // let torso = shape::Capsule {
        //     radius,
        //     depth: torso_depth,
        //     latitudes,
        //     longitudes,
        //     ..Default::default()
        // };

        // let arm = shape::Capsule {
        //     radius,
        //     depth: arm_depth,
        //     latitudes,
        //     longitudes,
        //     ..Default::default()
        // };

        // let leg = shape::Capsule {
        //     radius,
        //     depth: leg_depth,
        //     latitudes,
        //     longitudes,
        //     ..Default::default()
        // };

        // let arm_segment = shape::Capsule {
        //     depth: arm_segment_depth,
        //     radius,
        //     latitudes,
        //     longitudes,
        //     ..Default::default()
        // };

        // let upper_arm = commands.spawn((
        //     PbrBundle {
        //         mesh: meshes.add(arm_segment.into()),
        //         material: upper_arm_material,
        //         ..Default::default()
        //     },
        //     RigidBody::Fixed,
        //     Collider::capsule(
        //         Vec3::Y * (arm_segment_depth/2.), 
        //         Vec3::Y * (-arm_segment.depth/2.),
        //         radius
        //     ),
        // ));
    }
    
    let left_arm_cmp = StickmanArm::make_arm(
        SegmentInfo {
            length: arm_segment_len,
            thickness: radius,
        },
        SegmentInfo {
            length: arm_segment_len,
            thickness: radius,
        }, 
        ArmMotorParams {
            joint_axis: None,
            target_pos,
            target_vel,
            stiffness,
            damping,
        },
        &mut commands
    );
    let right_arm_cmp = StickmanArm::make_arm(
        SegmentInfo {
            length: arm_segment_len,
            thickness: radius,
        },
        SegmentInfo {
            length: arm_segment_len,
            thickness: radius,
        }, 
        ArmMotorParams {
            joint_axis: None,
            target_pos,
            target_vel,
            stiffness,
            damping,
        },
        &mut commands
    );

    let left_arm = commands.spawn(left_arm_cmp.clone());
    let right_arm = commands.spawn(right_arm_cmp.clone());

    let half_torso_depth = torso_depth/2.;
    let torso = commands.spawn((
        RigidBody::Fixed,
        Collider::capsule(Vec3::Y * -half_torso_depth, Vec3::Y * half_torso_depth, radius),
        Sleeping::default()
    )).id();

    let mut left_shoulder = SphericalJointBuilder::new()
        .local_anchor1(Vec3::new(radius/2., torso_len/2., 0.))
        .local_anchor2(Vec3::Y * (arm_segment_len/2.))
        .build();
    left_shoulder.set_contacts_enabled(false);

    let mut right_shoulder = SphericalJointBuilder::new()
        .local_anchor1(Vec3::new(-radius/2., torso_len/2., 0.))
        .local_anchor2(Vec3::Y * (arm_segment_len/2.))
        .build();
    right_shoulder.set_contacts_enabled(false);
    

    let mut upper_left_arm = commands.get_entity(left_arm_cmp.arm_segments[0]).unwrap();
    upper_left_arm.set_parent(torso)
        .insert(ImpulseJoint::new(torso, left_shoulder));
    
    let mut upper_right_arm = commands.get_entity(right_arm_cmp.arm_segments[0]).unwrap();
    upper_right_arm.set_parent(torso)
        .insert(ImpulseJoint::new(torso, right_shoulder));

}

pub fn test_update(
    mut multibody_joints: Query<(&mut MultibodyJoint, &mut Sleeping), With<StickmanArmSegment>>,
    keys: Res<Input<KeyCode>>,
) {
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };
        

    let default_limits = JointLimits::default();

    for (mut multibody_joint, mut sleeping) in multibody_joints.iter_mut() {
        let joint = &mut  multibody_joint.data;
        
        //revolute joint
        if joint.as_revolute().is_some() {
            let joint = unsafe { joint.as_revolute_mut().unwrap_unchecked() };
            let motor = joint.motor().unwrap();
            let target_vel = motor.target_vel;
            let current_target_pos = motor.target_pos;
            let stiffness = motor.stiffness;
            let damping = motor.damping;
            let limits = joint.limits().unwrap_or(&default_limits);
            
            let new_target_pos = f32::clamp(
                current_target_pos + dir,
                limits.min * (1. + damping),
                limits.max
            );

            if dir != 0. {
                sleeping.sleeping = false;
            }
            
            joint.set_motor(
                new_target_pos,
                target_vel,
                stiffness,
                damping
            );
        }

    }

}

pub fn update_joint_handles(
    mut arms: Query<&mut StickmanArm, Changed<StickmanArm>>,
    joint_handles: Query<&RapierMultibodyJointHandle>,
) {
    for mut arm in arms.iter_mut() {
        arm.joints.clear();

        for i in 0..arm.arm_segments.len() {
            let segment = arm.arm_segments[i];
            if let Ok(handle) = joint_handles.get(segment) {
                arm.joints.push(handle.clone());
            }
        }

    }
}
