use bevy::prelude::Transform;

use crate::StickmanBodyPart;

pub fn scale_limb(body_part: &StickmanBodyPart, transform: &mut Transform, scale_amount: f32) {
    transform.scale += scale_amount;
    // let new_length = (body_part.length() * (transform.scale * transform.local_y())).length();
    // println!("{}", new_length);
    let origin = transform.local_y() * body_part.length();
    // transform.translation += transform.translation - origin;
}