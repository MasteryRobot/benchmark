"""Auditable local design parameters; no hardware calibration is implied."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'towel_roll'
SPEC = {
    'schema_version': 1, 'id': 'bimanual_roll_towel',
    'title': '双灵巧手卷毛巾', 'suite': 'deformable_demos',
    'scene': 'scene.usda', 'robot': '../assets/dual_arm/robot.urdf',
    'workcell': '../tasks/workcell.usdc', 'hands': ['left', 'right'],
    'hand_chain_mapping': {'left': 'R', 'right': 'L'},
    'towel': {'width_m': .36, 'length_m': .40, 'thickness_m': .003,
              'mass_kg': .12, 'near_y_m': .32, 'table_height_m': .795,
              'grid_width': 25, 'grid_length': 97, 'color': [.12, .63, .68]},
    'physics': {
        'preview': 'CPU CAD mesh render; not Isaac sensor output',
        'exploration_backend': 'CPU width-coherent inextensible ribbon',
        'scope': 'Reduced-order deformable dynamics with gravity, bending, table friction and self-contact. Bilateral grasp constraints approximate pinches. No measured articulated finger contact forces; not full-physics training success.',
        'dt_s': .002, 'iterations': 35, 'table_friction': .65,
        'self_friction': .5,
    },
    'success': {'minimum_winding_turns': 1.2,
                'maximum_length_extent_m': .12, 'maximum_height_m': .10,
                'maximum_segment_strain': .08,
                'minimum_bilateral_grasp_seconds': 1.,
                'release_hold_seconds': .6,
                'maximum_released_speed_m_s': .035,
                'requires_articulated_contact_validation_for_training': True},
    'source': 'Locally designed task. Dimensions and material parameters are explicit uncalibrated design values.',
}
