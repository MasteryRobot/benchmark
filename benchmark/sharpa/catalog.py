"""Provisional task inventory and local implementation plans, 2026-10-29.

No source text was successfully retrieved in this environment. These identifiers
and URLs are candidates requiring source verification, NOT an official inventory.
Geometry, physics constants and predicates are local design choices.
"""
DATASET = 'https://huggingface.co/datasets/Sharpa-Robotics/SharpaDex-v1.0'
CES = 'https://www.sharpa.com/blogs/news/sharpa-unveils-its-first-autonomous-full-body-robot-with-human-dexterity-at-ces-2026'
IROS = 'https://sg.linkedin.com/company/sharpa-robotics'
DQ = 'https://www.sharpa.com/pages/sharpa-dq'
APPLE = 'https://www.sharpa.com/blogs/research/peeling-an-apple-with-63-dof-how-imcopilot-mode-vla-are-redefining-robotic-dexterity'

# Provisional local identifier | label | props | supports | missing process
DATASET_TASKS = '''
insert_batteries_into_charger|电池装入充电座|battery,battery|charger|
scrub_cup_interior|清洁杯内壁|brush|cup|surface dirt removal / brush compliance
insert_plug_into_socket|插入电源插头|plug|socket_board|electrical contacts / socket retention
assemble_gears_on_base|底板齿轮装配|gear,gear|board|gear meshing / axle constraints
clean_plate_with_eraser|擦净盘子|eraser|plate|surface dirt state
tighten_bottle_cap|拧紧瓶盖|cap,bottle||helical thread / torque completion
secure_coiled_cable_with_velcro_tie|魔术贴捆扎线圈|velcro,cable||flexible cable / adhesive fastening
collect_waste_into_bin|垃圾入桶|block,paper|bin|paper deformation
deal_playing_cards|逐张发牌|paper,paper,paper||specialized card scene
empty_dustpan_into_bin|簸箕倒入垃圾桶|dustpan,block,block|bin|
cover_ball_with_cup|杯子罩住球|cup,ball||
insert_knife_into_cutlery_bin|刀具插入收纳筒|knife|cutlery_bin|
place_ball_in_box|球放入盒内|ball|box|
place_ball_in_cup_then_cup_in_box|球入杯再连杯入盒|ball,cup|box|sequence / relative containment
place_cup_on_plate|杯子放到盘上|cup|plate|
place_knife_and_fork_on_plate|刀叉摆上餐盘|knife,fork|plate|
stack_two_plates|两个盘子堆叠|plate|plate|
remove_block_from_box|取出盒内积木|block|box|
reorient_cup_upright|把杯子扶正|cup||
fit_trash_bag_into_bin|垃圾袋套桶|bag|bin|deformable bag / rim fitting
turn_book_pages|翻书页|paper|book|paper hinges / page separation
fold_and_close_box|折叠合上纸盒|paper,box||foldable panels / closure
hang_garment_on_hanger|衣物挂衣架|garment,hanger||cloth / hanger draping
form_kraft_paper_box|牛皮纸成盒|paper||paper folds / creases
close_kraft_paper_box|封合牛皮纸盒|box,paper||paper flap constraints
fold_towel|折叠毛巾|towel||cloth bending / self collision
grind_medicine_with_mortar_and_pestle|研钵研磨药片|pestle,pill,pill|mortar|fragmentation / granular grinding
hammer_nails|锤击钉入木板|hammer,nail|board|penetration / material failure
insert_batteries_into_device|电池装入设备|battery,battery|charger|device latch / electrical contacts
iron_garment|熨烫衣物|iron,garment||cloth / heat / wrinkle state
clean_garment_with_lint_roller|滚筒清理衣物|lint_roller,garment||lint adhesion / cloth
stack_napkins_in_holder|餐巾放入纸巾座|napkin,napkin|holder|flexible napkins
nest_cups|杯子嵌套|cup|cup|tapered cup geometry
organize_objects_in_drawer|抽屉整理|block,pen|drawer|drawer slide / multi-object layout
pack_blueberries_in_bag|蓝莓装袋|blueberry,blueberry,blueberry,bag||deformable bag
pack_object_in_storage_bag|物品放入收纳袋|block,bag||bag opening / zipper
staple_paper|订书机装订纸张|stapler,paper||stapler hinge / staple deformation
relocate_tennis_ball|搬移网球|tennis_ball||
place_fruits_in_basket|水果装篮|fruit,apple|basket|
place_pens_in_holder|笔放入笔筒|pen,pen|holder|
place_coffee_filter_in_dripper|咖啡滤纸入滤杯|filter|dripper|deformable filter
insert_socket_module_into_board|插座模块入底板|socket_module|socket_board|snap-fit / latch
transfer_pills_between_cups|杯间转移药片|pill,pill|cup,cup|
pry_nails_from_board|拔出木板钉子|hammer,nail|board|nail embedding / extraction friction
reorient_and_relocate_carton|翻转搬移纸盒|carton||
transfer_liquid_with_dropper|滴管转移液体|dropper|cup,cup|fluid / suction pressure
transfer_salt_between_cups_with_spoon|勺子杯间转移盐|spoon|cup,cup|granular medium / volume transfer
scoop_salt_from_jar_into_cup|从罐中舀盐入杯|spoon|jar,cup|granular medium / volume transfer
sort_utensils|分类摆放餐具|knife,fork,spoon|tray,tray|
reposition_and_stack_blocks|移动并堆叠积木|block,block,block||
sweep_waste_into_dustpan|扫垃圾入簸箕|broom,block,block|dustpan|sweeping contact / collection predicate
seal_box_with_tape|胶带封箱|tape|box|adhesion / flexible tape
place_books_on_shelf|书本上架|book,book|shelf|shelf access orientation
collect_toys_into_basket|玩具装篮|toy,toy|basket|
rotate_bottle_cap|旋转瓶盖|cap,bottle||helical coupling
unscrew_bottle_cap|旋开瓶盖|cap,bottle||helical coupling / release
clean_knife_with_cloth|用布擦刀具|knife,cloth||cloth / surface cleanliness
play_tic_tac_toe|井字棋落子|block,block,block|board|game rules / opponent / mark identity
tomato_boxed_lunch_packaging|番茄盒饭打包|tomato,tofu|box|food compliance / packaging closure
'''.strip()


def dataset_rows():
    rows = []
    for line in DATASET_TASKS.splitlines():
        name, title, props, supports, missing = line.split('|')
        rows.append(dict(source_task=name, title=title, props=props.split(','),
                         supports=supports.split(',') if supports else [], missing=missing))
    if len(rows) != 59 or len({r['source_task'] for r in rows}) != 59:
        raise ValueError('Provisional inventory must contain 59 distinct local identifiers')
    return rows
