use bevy::prelude::*;
use bevy_flycam::FlyCam;
use bevy_rapier3d::{prelude::*, rapier::prelude::{JointLimits, MotorModel}};
use crate::stickman::{SegmentInfo, StickmanCommandsExt};

pub fn test_update(
    mut impulse_joints: Query<(&mut MultibodyJoint, &mut Sleeping)>,
    keys: Res<Input<KeyCode>>,
) {
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };

    let default_limits = JointLimits::default();

    for (mut impulse_joint, mut sleeping) in impulse_joints.iter_mut() {
        let joint = &mut  impulse_joint.data;
        
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

pub fn spawn_cubes(
    keys: Res<Input<KeyCode>>,
    cam_transform: Query<&Transform, With<FlyCam>>,
    mut commands: Commands,
) {
    // spawning cubes
    if keys.just_pressed(KeyCode::J) {
        if let Ok(transform) = cam_transform.get_single() {
            commands.spawn((
                Collider::cuboid(0.1, 0.1, 0.1),
                RigidBody::Dynamic,
                GlobalTransform::from(transform.clone()),
                ColliderMassProperties::Mass(1.)
            ));
        }
    }
}


pub fn stickman_setup(
    mut commands: Commands
) {
    let scale = 1.;

    let radius = 0.03_f32 * scale;
    let torso_depth = 0.6_f32 * scale;
    let torso_len = torso_depth + radius*2.;
    let arm_depth = torso_depth;
    let upper_arm_len = (arm_depth*0.7) + radius*2.;
    let lower_arm_len = (arm_depth*0.49) + radius*2.;
    let thickness = radius;

    //motor params
    let target_pos = f32::to_radians(0.);
    let target_vel = f32::to_radians(30.);
    let stiffness = 1.;
    let damping = 0.03;

    //torso
    let fixed_movement = commands.spawn(RigidBody::Fixed).id();
    let torso = commands.spawn((
        RigidBody::Dynamic,
        ImpulseJoint::new(fixed_movement, FixedJoint::new())
    )).id();
    commands.add_segment_physics(
        torso,
        SegmentInfo {
            length: torso_len,
            thickness
        }
    );

    //upper arm
    let upper_arm = commands.spawn(RigidBody::Dynamic).id();
    commands.add_segment_physics(
        upper_arm,
        SegmentInfo {
            length: upper_arm_len,
            thickness
        }
    );

    //lower arm
    let lower_arm = commands.spawn(RigidBody::Dynamic).id();
    commands.add_segment_physics(
        lower_arm,
        SegmentInfo {
            length: lower_arm_len,
            thickness
        }
    );


    //joints
    let mut shoulder = SphericalJoint::new();
        shoulder.set_contacts_enabled(false);
        let shoulder: GenericJoint = shoulder.into();
    let mut elbow = RevoluteJointBuilder::new(Vec3::Z)
        .limits([0., 150_f32.to_radians()])
        .motor(target_pos, target_vel, stiffness, damping)
        .motor_model(MotorModel::ForceBased)
        .build();
        elbow.set_contacts_enabled(false);

    commands.create_arm(
        upper_arm,
        lower_arm,
        Some(torso),
        elbow,
        Some(shoulder),
        true
    );
    
}
