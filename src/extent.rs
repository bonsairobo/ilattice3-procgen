use ilattice3::Extent;

pub fn push_extents_apart(r1: Extent, r2: Extent) -> (Extent, Extent) {
    let (push_v, direction) = Extent::penetrations(&r1, &r2).min_vector();

    // Only push in positive directions to prevent infinite cycles.
    if direction.is_negative() {
        (r1, r2 - push_v)
    } else {
        (r1 + push_v, r2)
    }
}

pub fn resolve_extent_overlaps(rooms: &mut [Extent]) {
    let num_rooms = rooms.len();
    loop {
        // PERF: N^2 gets slow for >1000 rooms
        let mut all_rooms_separated = true;
        for i in 0..num_rooms {
            for j in i + 1..num_rooms {
                let (r1, r2) = (rooms[i], rooms[j]);
                let int = r1.intersection(&r2);

                if int.is_empty() {
                    continue;
                }

                all_rooms_separated = false;
                let (r1, r2) = push_extents_apart(r1, r2);
                debug_assert!(r1.intersection(&r2).is_empty());
                rooms[i] = r1;
                rooms[j] = r2;
            }
        }

        if all_rooms_separated {
            break;
        }
    }
}
