# 场景能力对照表

以下为本地原型清单，官方来源均待核实。入口数不代表独立官方 demo 数或成功率。

共 72 个入口：25 个有近似判定，47 个只有道具与布局。所有 Isaac 物理实测仍未完成。

发牌在演示候选与桌面清单中重复；其它可能的重叠须按官方视频去重。

| 场景 ID | 名称 | 支持程度 | 尚缺能力 / 限制 |
| --- | --- | --- | --- |
| sharpa_card_dealing | 多张纸牌分发（本地原型） | 刚体近似判定（未实测） | Five rigid cards; bending, shuffling and full deck friction are not modeled. |
| sharpa_windmill_assembly | 风车转子与套环装配（本地原型） | 刚体近似判定（未实测） | Horizontal axle insertion surrogate; no threads, snap-fit or powered windmill.; Free rigid rotor and collar remain separate after insertion; no automatic welding. |
| sharpa_camera_photography | 双手持相机拍照（本地原型） | 刚体近似判定（未实测） | Capture is an explicit action returning rendered RGB; shutter mechanics and autofocus are not modeled.; Success checks held pose and an actual nonempty captured frame, not image aesthetic quality. |
| sharpa_table_tennis | 桌面缩比乒乓球回击（本地原型） | 刚体近似判定（未实测） | Scaled single-return drill on the existing workcell; not a regulation table or an opponent.; No aerodynamic drag or spin-dependent Magnus model. Contact impulses use the physics engine. |
| sharpa_baton_catching | 运动棒接取（本地原型） | 刚体近似判定（未实测） | Deterministic launch after reset; no external throwing robot or learned catcher. |
| sharpa_dex_insert_batteries_into_charger | 电池装入充电座（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_scrub_cup_interior | 清洁杯内壁（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: surface dirt removal / brush compliance.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_insert_plug_into_socket | 插入电源插头（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: electrical contacts / socket retention.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_assemble_gears_on_base | 底板齿轮装配（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: gear meshing / axle constraints.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_clean_plate_with_eraser | 擦净盘子（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: surface dirt state.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_tighten_bottle_cap | 拧紧瓶盖（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: helical thread / torque completion.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_secure_coiled_cable_with_velcro_tie | 魔术贴捆扎线圈（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: flexible cable / adhesive fastening.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_collect_waste_into_bin | 垃圾入桶（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: paper deformation.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_deal_playing_cards | 逐张发牌（本地仿真原型） | 刚体近似判定（未实测） | Five rigid cards; bending, shuffling and full deck friction are not modeled. |
| sharpa_dex_empty_dustpan_into_bin | 簸箕倒入垃圾桶（本地仿真原型） | 刚体近似判定（未实测） | Checks deposited rigid waste; does not require a particular pan trajectory. |
| sharpa_dex_cover_ball_with_cup | 杯子罩住球（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_insert_knife_into_cutlery_bin | 刀具插入收纳筒（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_place_ball_in_box | 球放入盒内（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_place_ball_in_cup_then_cup_in_box | 球入杯再连杯入盒（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: sequence / relative containment.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_place_cup_on_plate | 杯子放到盘上（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_place_knife_and_fork_on_plate | 刀叉摆上餐盘（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_stack_two_plates | 两个盘子堆叠（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_remove_block_from_box | 取出盒内积木（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_reorient_cup_upright | 把杯子扶正（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_fit_trash_bag_into_bin | 垃圾袋套桶（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: deformable bag / rim fitting.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_turn_book_pages | 翻书页（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: paper hinges / page separation.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_fold_and_close_box | 折叠合上纸盒（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: foldable panels / closure.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_hang_garment_on_hanger | 衣物挂衣架（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: cloth / hanger draping.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_form_kraft_paper_box | 牛皮纸成盒（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: paper folds / creases.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_close_kraft_paper_box | 封合牛皮纸盒（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: paper flap constraints.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_fold_towel | 折叠毛巾（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: cloth bending / self collision.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_grind_medicine_with_mortar_and_pestle | 研钵研磨药片（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: fragmentation / granular grinding.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_hammer_nails | 锤击钉入木板（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: penetration / material failure.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_insert_batteries_into_device | 电池装入设备（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: device latch / electrical contacts.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_iron_garment | 熨烫衣物（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: cloth / heat / wrinkle state.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_clean_garment_with_lint_roller | 滚筒清理衣物（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: lint adhesion / cloth.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_stack_napkins_in_holder | 餐巾放入纸巾座（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: flexible napkins.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_nest_cups | 杯子嵌套（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: tapered cup geometry.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_organize_objects_in_drawer | 抽屉整理（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: drawer slide / multi-object layout.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_pack_blueberries_in_bag | 蓝莓装袋（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: deformable bag.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_pack_object_in_storage_bag | 物品放入收纳袋（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: bag opening / zipper.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_staple_paper | 订书机装订纸张（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: stapler hinge / staple deformation.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_relocate_tennis_ball | 搬移网球（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_place_fruits_in_basket | 水果装篮（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_place_pens_in_holder | 笔放入笔筒（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_place_coffee_filter_in_dripper | 咖啡滤纸入滤杯（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: deformable filter.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_insert_socket_module_into_board | 插座模块入底板（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: snap-fit / latch.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_transfer_pills_between_cups | 杯间转移药片（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_pry_nails_from_board | 拔出木板钉子（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: nail embedding / extraction friction.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_reorient_and_relocate_carton | 翻转搬移纸盒（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_transfer_liquid_with_dropper | 滴管转移液体（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: fluid / suction pressure.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_transfer_salt_between_cups_with_spoon | 勺子杯间转移盐（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: granular medium / volume transfer.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_scoop_salt_from_jar_into_cup | 从罐中舀盐入杯（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: granular medium / volume transfer.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_sort_utensils | 分类摆放餐具（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_reposition_and_stack_blocks | 移动并堆叠积木（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_sweep_waste_into_dustpan | 扫垃圾入簸箕（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: sweeping contact / collection predicate.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_seal_box_with_tape | 胶带封箱（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: adhesion / flexible tape.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_place_books_on_shelf | 书本上架（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: shelf access orientation.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_collect_toys_into_basket | 玩具装篮（本地仿真原型） | 刚体近似判定（未实测） | 局部位姿/接触代理判定；未验证可达性或完整操作。 |
| sharpa_dex_rotate_bottle_cap | 旋转瓶盖（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: helical coupling.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_unscrew_bottle_cap | 旋开瓶盖（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: helical coupling / release.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_clean_knife_with_cloth | 用布擦刀具（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: cloth / surface cleanliness.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_play_tic_tac_toe | 井字棋落子（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: game rules / opponent / mark identity.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_dex_tomato_boxed_lunch_packaging | 番茄盒饭打包（本地仿真原型） | 仅场景道具（无任务判定） | Not implemented: food compliance / packaging closure.; Physical props and reset/control available; no task-success predicate until missing process is implemented. |
| sharpa_demo_pen_spinning | 手内转笔（本地原型） | 仅场景道具（无任务判定） | Pen geometry and two-hand contact observation available; continuous in-hand turn counting and grasp transitions are not implemented. |
| sharpa_demo_bartending | 调饮与摇壶（本地原型） | 仅场景道具（无任务判定） | Rigid vessels and tools only; no fluid, pouring volume or shaker seal. |
| sharpa_demo_gpu_installation | 显卡插装（本地原型） | 刚体近似判定（未实测） | Rigid card insertion alignment only; connector resistance, latch and electrical function are not modeled. |
| sharpa_demo_apple_peeling | 苹果削皮（本地原型） | 仅场景道具（无任务判定） | No peel separation, cutting, deformable skin or volume-loss metric. |
| sharpa_demo_dessert_service | 甜品取杯与加料（本地原型） | 仅场景道具（无任务判定） | Local service layout; no ice cream/fluid dispensing, order system or vendor equipment. |
| sharpa_demo_paper_cutting | 剪纸（本地原型） | 仅场景道具（无任务判定） | Two independent scissor blades are shown as tool components; no revolute scissors or paper fracture. |
| sharpa_demo_needle_threading | 穿针引线（本地原型） | 仅场景道具（无任务判定） | Eyelet geometry and cable proxy only; no deformable thread/needle insertion predicate. |
| sharpa_demo_screwdriver_use | 螺丝刀拧装（本地原型） | 仅场景道具（无任务判定） | Rigid tools only; thread coupling, torque and insertion depth not implemented. |
