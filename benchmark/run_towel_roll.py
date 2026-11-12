"""Build, preview, or explore the bimanual towel-roll demo."""
import argparse
from pathlib import Path
from datetime import datetime, timezone

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('task', nargs='?', choices=['bimanual_roll_towel'])
    p.add_argument('--backend', choices=['preview', 'isaac'], default='preview')
    p.add_argument('--prepare-only', action='store_true', help='author surface-deformable USD; does not run physics')
    p.add_argument('--joint-interaction', action='store_true', help='run the 8 mm GPU joint-only interaction probe')
    p.add_argument('--variant', default='baseline')
    p.add_argument('--gui', action='store_true')
    p.add_argument('--output', type=Path)
    p.add_argument('--actions', type=Path)
    p.add_argument('--explore',action='store_true',help='run CPU flex exploration after rebuilding')
    p.add_argument('--duration',type=float,default=1.0)
    p.add_argument('--turns',type=float,default=1.45)
    args=p.parse_args()
    if args.actions:
        p.error('Towel articulated action replay is not implemented')
    if args.joint_interaction:
        if args.backend != 'isaac' or args.prepare_only or args.explore or args.gui or args.variant != 'baseline':
            p.error('--joint-interaction requires --backend isaac and uses interaction_config.json')
        from towel_roll.gpu_interaction import run
        from towel_roll.config import FOLDER
        output=args.output or FOLDER/'attempts'/('joint_gpu_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f'))
        return run(output)
    if args.backend == 'isaac':
        if args.explore:
            p.error('--explore is the separate CPU approximation')
        from towel_roll.prepare_isaac import prepare
        from towel_roll.config import FOLDER
        output = args.output or FOLDER / 'attempts' / ('isaac_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f') + '.json')
        return prepare(output, args.variant, args.prepare_only, args.gui)
    if args.prepare_only or args.output or args.gui or args.variant != 'baseline':
        p.error('These options require --backend isaac')
    from towel_roll.build import build
    build()
    if args.explore:
        from towel_roll.explore import main as explore_main
        import sys
        sys.argv=[sys.argv[0],'--duration',str(args.duration),'--turns',str(args.turns)]
        explore_main()
    return 0

if __name__=='__main__': raise SystemExit(main())
