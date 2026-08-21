RIGHT_CR5_LINK_TO_ADJACENT_LINKS = {
    "base_link": ["Link1"],
    "Link1": ["base_link", "Link2"],
    "Link2": ["Link1", "Link3"],
    "Link3": ["Link2", "Link4"],
    "Link4": ["Link3", "Link5"],
    "Link5": ["Link4", "Link6"],
    "Link6": ["Link5", "right_hand_base_link"],
    "right_hand_base_link": [
        "Link6",
        "right_thumb_metacarpals_base1",
        "right_index_metacarpals",
        "right_middle_metacarpals",
        "right_ring_metacarpals",
        "right_pinky_metacarpals"
    ],
    "right_thumb_metacarpals_base1": [
        "right_hand_base_link",
        "right_thumb_metacarpals_base2"
    ],
    "right_thumb_metacarpals_base2": [
        "right_thumb_metacarpals_base1",
        "right_thumb_metacarpals"
    ],
    "right_thumb_metacarpals": [
        "right_thumb_metacarpals_base2",
        "right_thumb_proximal"
    ],
    "right_thumb_proximal": [
        "right_thumb_metacarpals",
        "right_thumb_F80"
    ],
    "right_thumb_F80": ["right_thumb_proximal"],
    "right_index_metacarpals": [
        "right_hand_base_link",
        "right_index_proximal"
    ],
    "right_index_proximal": [
        "right_index_metacarpals",
        "right_index_middle"
    ],
    "right_index_middle": [
        "right_index_proximal",
        "right_index_F80"
    ],
    "right_index_F80": ["right_index_middle"],
    "right_middle_metacarpals": [
        "right_hand_base_link",
        "right_middle_proximal"
    ],
    "right_middle_proximal": [
        "right_middle_metacarpals",
        "right_middle_middle"
    ],
    "right_middle_middle": [
        "right_middle_proximal",
        "right_middle_F80"
    ],
    "right_middle_F80": ["right_middle_middle"],
    "right_ring_metacarpals": [
        "right_hand_base_link",
        "right_ring_proximal"
    ],
    "right_ring_proximal": [
        "right_ring_metacarpals",
        "right_ring_middle"
    ],
    "right_ring_middle": [
        "right_ring_proximal",
        "right_ring_F80"
    ],
    "right_ring_F80": ["right_ring_middle"],
    "right_pinky_metacarpals": [
        "right_hand_base_link",
        "right_pinky_proximal"
    ],
    "right_pinky_proximal": [
        "right_pinky_metacarpals",
        "right_pinky_middle"
    ],
    "right_pinky_middle": [
        "right_pinky_proximal",
        "right_pinky_F80"
    ],
    "right_pinky_F80": ["right_pinky_middle"],
}

LEFT_CR5_LINK_TO_ADJACENT_LINKS = {
    "base_link": ["Link1"],
    "Link1": ["base_link", "Link2"],
    "Link2": ["Link1", "Link3"],
    "Link3": ["Link2", "Link4"],
    "Link4": ["Link3", "Link5"],
    "Link5": ["Link4", "Link6"],
    "Link6": ["Link5", "left_hand_base_link"],
    "left_hand_base_link": [
        "Link6",
        "left_thumb_metacarpals_base1",
        "left_index_metacarpals",
        "left_middle_metacarpals",
        "left_ring_metacarpals",
        "left_pinky_metacarpals"
    ],
    "left_thumb_metacarpals_base1": [
        "left_hand_base_link",
        "left_thumb_metacarpals_base2"
    ],
    "left_thumb_metacarpals_base2": [
        "left_thumb_metacarpals_base1",
        "left_thumb_metacarpals"
    ],
    "left_thumb_metacarpals": [
        "left_thumb_metacarpals_base2",
        "left_thumb_proximal"
    ],
    "left_thumb_proximal": [
        "left_thumb_metacarpals",
        "left_thumb_F80"
    ],
    "left_thumb_F80": ["left_thumb_proximal"],
    "left_index_metacarpals": [
        "left_hand_base_link",
        "left_index_proximal"
    ],
    "left_index_proximal": [
        "left_index_metacarpals",
        "left_index_middle"
    ],
    "left_index_middle": [
        "left_index_proximal",
        "left_index_F80"
    ],
    "left_index_F80": ["left_index_middle"],
    "left_middle_metacarpals": [
        "left_hand_base_link",
        "left_middle_proximal"
    ],
    "left_middle_proximal": [
        "left_middle_metacarpals",
        "left_middle_middle"
    ],
    "left_middle_middle": [
        "left_middle_proximal",
        "left_middle_F80"
    ],
    "left_middle_F80": ["left_middle_middle"],
    "left_ring_metacarpals": [
        "left_hand_base_link",
        "left_ring_proximal"
    ],  
    "left_ring_proximal": [
        "left_ring_metacarpals",
        "left_ring_middle"
    ],
    "left_ring_middle": [
        "left_ring_proximal",
        "left_ring_F80"
    ],
    "left_ring_F80": ["left_ring_middle"],
    "left_pinky_metacarpals": [
        "left_hand_base_link",
        "left_pinky_proximal"
    ],
    "left_pinky_proximal": [
        "left_pinky_metacarpals",
        "left_pinky_F80"
    ],
    "left_pinky_F80": ["left_pinky_middle"],
}