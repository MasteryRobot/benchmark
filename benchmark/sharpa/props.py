"""Local analytic props with open collision geometry (not manufacturer CAD)."""
import math
from .recipes import cube, cylinder, shape, ring


def open_box(w=.15, d=.12, h=.08, t=.005, color=(.38, .51, .56)):
    return [cube([w, d, t], [0, 0, t/2], color),
            cube([t, d, h], [-w/2+t/2, 0, h/2], color),
            cube([t, d, h], [w/2-t/2, 0, h/2], color),
            cube([w-2*t, t, h], [0, -d/2+t/2, h/2], color),
            cube([w-2*t, t, h], [0, d/2-t/2, h/2], color)]


def cup(r=.038, h=.085, t=.003, color=(.78, .83, .85)):
    parts = [cylinder(r, t, [0, 0, t/2], color)]
    walls = ring(r-t/2, t, h-t, color)
    for p in walls:
        p['position'][2] = (h+t)/2
    return parts + walls


def prop(kind):
    # Returns parts, mass and half-height for table-supported local origins.
    # Hollow props use origins on their underside; solid props use centers.
    if kind in ('cup', 'jar', 'holder', 'dripper', 'mortar'):
        r, h = {'cup':(.038,.085), 'jar':(.045,.09), 'holder':(.035,.075),
                'dripper':(.05,.07), 'mortar':(.065,.04)}[kind]
        return cup(r,h), .10, 0
    if kind in ('box','bin','basket','tray','drawer','cutlery_bin'):
        w,d,h = {'box':(.18,.14,.09), 'bin':(.19,.17,.15), 'basket':(.21,.17,.10),
                 'tray':(.19,.15,.018), 'drawer':(.23,.16,.055), 'cutlery_bin':(.10,.09,.12)}[kind]
        return open_box(w,d,h), .18, 0
    if kind == 'plate':
        return cup(.09,.012,.003,[.84,.88,.90]), .14, 0
    if kind in ('ball','tennis_ball','fruit','apple','tomato','toy','blueberry','pill'):
        radius = {'ball':.018,'tennis_ball':.033,'fruit':.032,'apple':.036,'tomato':.03,
                  'toy':.025,'blueberry':.009,'pill':.006}[kind]
        color = [.8,.22,.13] if kind in ('apple','fruit','tomato') else [.32,.63,.83]
        return [shape('sphere',[radius], color=color)], max(.001,radius**3*5000), radius
    if kind in ('block','eraser','carton','book','gpu','tofu','soap','socket_module','plug'):
        size = {'block':[.035,.035,.035], 'eraser':[.045,.065,.022], 'carton':[.065,.055,.13],
                'book':[.10,.14,.018], 'gpu':[.18,.025,.10], 'tofu':[.06,.05,.035],
                'soap':[.045,.045,.10], 'socket_module':[.025,.03,.018], 'plug':[.03,.038,.018]}[kind]
        parts = [cube(size)]
        if kind == 'plug':
            parts += [cube([.003,.018,.004],[x,.027,0],[.65,.67,.7]) for x in [-.008,.008]]
        if kind == 'gpu':
            parts += [cube([.12,.002,.01],[0,0,-.055],[.8,.6,.15])]
        return parts, .07, .060 if kind == 'gpu' else size[2]/2
    if kind in ('battery','pen','tube','pestle','dropper','bottle','cap','nail'):
        r,h = {'battery':(.007,.05), 'pen':(.006,.15), 'tube':(.012,.10), 'pestle':(.017,.13),
               'dropper':(.006,.11), 'bottle':(.028,.12), 'cap':(.020,.016), 'nail':(.003,.045)}[kind]
        return [cylinder(r,h)], .025 if kind != 'bottle' else .12, h/2
    if kind in ('knife','fork','spoon','brush','peeler','screwdriver','marker','spatula','whisk'):
        parts = [cube([.018,.09,.014],[0,-.04,0],[.19,.26,.29])]
        if kind in ('knife','spatula'):
            parts += [cube([.028 if kind=='knife' else .05,.09,.003],[0,.05,0],[.67,.70,.73])]
        elif kind == 'fork':
            parts += [cube([.003,.06,.003],[x,.05,0],[.67,.70,.73]) for x in [-.009,-.003,.003,.009]]
        elif kind == 'spoon':
            parts += [cylinder(.022,.003,[0,.04,0])]
        elif kind == 'peeler':
            parts += [cube([.05,.006,.006],[0,.025,0]),cube([.006,.03,.006],[-.022,.01,0]),cube([.006,.03,.006],[.022,.01,0])]
        elif kind in ('screwdriver','marker'):
            parts += [cylinder(.003,.085,[0,.035,0],axis='Y')]
        else:
            parts += [cube([.045,.055,.018],[0,.035,0],[.5,.35,.15])]
        return parts,.045,.009
    if kind in ('hammer','lint_roller','iron','stapler'):
        if kind=='hammer':
            return [cube([.018,.13,.018]),cube([.075,.032,.035],[0,.055,0])],.18,.0175
        if kind=='lint_roller':
            return [cube([.015,.09,.015],[0,-.04,0]),cylinder(.024,.07,[0,.035,0],axis='X')],.08,.024
        if kind=='iron':
            return [cube([.07,.12,.014]),cube([.025,.085,.025],[0,0,.025])],.3,.007
        return [cube([.025,.09,.012]),cube([.025,.09,.012],[0,0,.025])],.12,.006
    if kind in ('cloth','garment','towel','napkin','paper','bag','filter','tape','velcro'):
        size = {'cloth':[.14,.12,.002], 'garment':[.24,.19,.003], 'towel':[.22,.16,.003],
                'napkin':[.09,.09,.001], 'paper':[.20,.16,.001], 'bag':[.16,.14,.001],
                'filter':[.09,.09,.001], 'tape':[.12,.016,.001], 'velcro':[.10,.015,.002]}[kind]
        return [cube(size,color=[.79,.67,.44])],.01,size[2]/2
    if kind in ('gear','collar'):
        parts = ring(.022,.010,.008,[.75,.56,.20])
        if kind=='gear':
            for i in range(12):
                a=i*math.pi/6
                p=cube([.009,.007,.008],[.030*math.cos(a),.030*math.sin(a),0],[.75,.56,.20])
                p['yaw_deg']=math.degrees(a);parts.append(p)
        return parts,.025,.004
    if kind in ('board','charger','socket_board','shelf','hanger','dustpan','broom','cable','lamp','tissue_box','shaker'):
        if kind=='board':return [cube([.18,.14,.01])],.2,.005
        if kind=='charger':
            parts=open_box(.075,.08,.018)
            parts += [cube([.004,.07,.016],[0,0,.008])]
            return parts,.12,0
        if kind=='socket_board':return open_box(.10,.08,.025),.15,0
        if kind=='shelf':return open_box(.24,.10,.18),.25,0
        if kind=='hanger':return [cube([.18,.008,.01]),cube([.008,.05,.01],[0,.025,0])],.035,.005
        if kind=='dustpan':return open_box(.13,.10,.018)[:4],.06,0
        if kind=='broom':return [cube([.012,.14,.012]),cube([.09,.035,.025],[0,.07,0])],.08,.0125
        if kind=='cable':return ring(.04,.006,.006,[.13,.14,.17]),.035,.003
        if kind=='lamp':return [cylinder(.045,.01,[0,0,.005]),cylinder(.007,.12,[0,0,.07]),cube([.07,.045,.02],[0,0,.14])],.15,0
        if kind=='tissue_box':return open_box(.11,.08,.06),.06,0
        return cup(.035,.12),.10,0
    raise ValueError(f'Unknown prop type: {kind}')
