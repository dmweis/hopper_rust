use nalgebra::{ Point3, distance };

#[derive(Debug, PartialEq, Clone)]
pub(crate) struct LegPositions {
    pub left_front: Point3<f32>,
    pub left_middle: Point3<f32>,
    pub left_rear: Point3<f32>,
    pub right_front: Point3<f32>,
    pub right_middle: Point3<f32>,
    pub right_rear: Point3<f32>,
}

impl LegPositions {
    fn new(
        left_front: Point3<f32>,
        left_middle: Point3<f32>,
        left_rear: Point3<f32>,
        right_front: Point3<f32>,
        right_middle: Point3<f32>,
        right_rear: Point3<f32>,
    ) -> LegPositions {
        LegPositions {
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        }
    }
}

trait MoveTowards {
    type Item;

    fn move_towards(&self, target: &Self, max_move: &f32) -> (Self::Item, bool);
}

impl MoveTowards for Point3<f32> {
    type Item = Point3<f32>;

    fn move_towards(&self, target: &Point3<f32>, max_move: &f32) -> (Point3<f32>, bool) {
        if self == target {
            return (target.clone(), false);
        }
        let distance = distance(self, target);
        if &distance <= max_move {
            // TODO: Think about if this should be true or false
            return (target.clone(), true)
        }
        let vector = target - self;
        (self + vector.normalize() * *max_move, true)
    }
}

impl MoveTowards for LegPositions {
    type Item = LegPositions;

    fn move_towards(&self, target: &LegPositions, max_move: &f32) -> (LegPositions, bool) {
        let (left_front, lf_moved) = self.left_front.move_towards(&target.left_front, max_move);
        let (left_middle, lm_moved) = self.left_middle.move_towards(&target.left_middle, max_move);
        let (left_rear, lr_moved) = self.left_rear.move_towards(&target.left_rear, max_move);
        let (right_front, rf_moved) = self.right_front.move_towards(&target.right_front, max_move);
        let (right_middle, rm_moved) = self.right_middle.move_towards(&target.right_middle, max_move);
        let (right_rear, rr_moved) = self.right_rear.move_towards(&target.right_rear, max_move);
        let leg_positions = LegPositions::new(
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear
        );
        let moved = lf_moved || lm_moved || lr_moved || rf_moved || rm_moved || rr_moved;
        (leg_positions, moved)
    }
}

pub(crate) struct OptionalLegPositions {
    pub left_front: Option<Point3<f32>>,
    pub left_middle: Option<Point3<f32>>,
    pub left_rear: Option<Point3<f32>>,
    pub right_front: Option<Point3<f32>>,
    pub right_middle: Option<Point3<f32>>,
    pub right_rear: Option<Point3<f32>>,
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn move_point_towards_full_step() {
        let start = Point3::new(0_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 1_f32, 1_f32);
        let (new, moved) = start.move_towards(&target, &distance(&start, &target));
        assert!(moved);
        assert_eq!(new, target);
    }

    #[test]
    fn move_point_towards_half() {
        let start = Point3::new(0_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 0_f32, 0_f32);
        let (new, moved) = start.move_towards(&target, &(distance(&start, &target)/2.0));
        assert!(moved);
        assert_eq!(new, Point3::new(0.5, 0.0, 0.0));
    }

    #[test]
    fn move_point_towards_not_move() {
        let start = Point3::new(1_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 0_f32, 0_f32);
        let (new, moved) = start.move_towards(&target, &10.0);
        assert!(!moved);
        assert_eq!(new, target);
    }

    #[test]
    fn move_legs_towards_full_step() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let from = LegPositions::new(
            a.clone(),
            a.clone(),
            a.clone(),
            a.clone(),
            a.clone(),
            a.clone(),
        );
        let b = Point3::new(1.0, 0.0, 0.0);
        let to = LegPositions::new(
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
        );
        let (new, moved) = from.move_towards(&to, &1.0);
        assert!(moved);
        assert_eq!(to, new);
    }

    #[test]
    fn move_legs_towards_multiple_steps() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(2.0, 0.0, 0.0);
        let start = LegPositions::new(
            a.clone(),
            a.clone(),
            a.clone(),
            a.clone(),
            a.clone(),
            a.clone(),
        );
        let middle = LegPositions::new(
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
        );
        let target = LegPositions::new(
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
            b.clone(),
            c.clone(),
        );
        let (new, moved) = start.move_towards(&target, &1.0);
        assert!(moved);
        assert_eq!(middle, new);
        let (new, moved) = new.move_towards(&target, &1.0);
        assert!(moved);
        assert_eq!(target, new);
        let (new, moved) = new.move_towards(&target, &1.0);
        assert!(!moved);
        assert_eq!(target, new);
    }

    #[test]
    fn move_legs_towards_moves_correct_legs() {
        let start = LegPositions::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );
        
        let target = LegPositions::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
        );
        let mut step = start;
        let mut keep_moving = true;
        let mut counter = 0;
        while keep_moving {
            let (new, moved) = step.move_towards(&target, &1.0);
            step = new;
            keep_moving = moved;
            counter += 1;
        }
        assert_eq!(counter, 7);
        assert_eq!(target, step);
    }
}