"""Search cached per-object licenses and descriptions for benchmark prop classes."""
import gzip,json,re,collections,heapq
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'reports/full_asset_replacement_20261031'
TERMS={
 'battery':['aa battery','battery aa','alkaline battery','duracell','battery'],
 'charger':['battery charger','aa charger','battery holder','battery compartment'],
 'brush':['dish brush','bottle brush','scrub brush','cleaning brush','scrubbing brush'],
 'plug':['electric plug','electrical plug','power plug','plug'],
 'socket_board':['power strip','extension socket','electrical socket','power socket','wall socket'],
 'gear':['spur gear','cogwheel','cog wheel','gear'],
 'board':['wooden board','wood plank','wooden plank','wood board','plank'],
 'eraser':['eraser','rubber eraser'],
 'cap':['bottle cap','screw cap','bottle lid'],
 'bottle':['plastic bottle','water bottle','bottle'],
 'velcro':['velcro','cable tie','velcro strap'],
 'cable':['coiled cable','coil cable','power cable','electric cable','cable'],
 'block':['wooden cube','wood cube','wooden block','wood block','building block'],
 'paper':['sheet of paper','paper sheet','blank paper','a4 paper','paper'],
 'bin':['trash bin','trash can','garbage bin','garbage can','waste bin'],
 'dustpan':['dustpan','dust pan'],
 'ball':['ping pong ball','ping-pong ball','table tennis ball','rubber ball','plastic ball'],
 'cutlery_bin':['utensil holder','cutlery holder','utensil crock','cutlery container','pencil holder'],
 'box':['open cardboard box','cardboard box','open box','carton box'],
 'bag':['plastic bag','storage bag','ziplock bag','zip bag','paper bag'],
 'book':['open book','book'],
 'garment':['t-shirt','tshirt','tee shirt','shirt'],
 'hanger':['clothes hanger','coat hanger','hanger'],
 'towel':['towel','hand towel','washcloth'],
 'pestle':['pestle','mortar and pestle','mortar pestle'],
 'pill':['pill','tablet medicine','capsule medicine','medicine tablet'],
 'mortar':['mortar and pestle','mortar pestle','mortar bowl','mortar'],
 'nail':['iron nail','metal nail','steel nail','nail'],
 'iron':['clothes iron','clothing iron','steam iron','electric iron','ironing iron','flat iron'],
 'lint_roller':['lint roller','lint remover'],
 'napkin':['napkin','serviette'],
 'holder':['pencil holder','pen holder','napkin holder'],
 'pen':['ballpoint pen','ball point pen','pen'],
 'drawer':['drawer','desk drawer'],
 'blueberry':['blueberry','blueberries'],
 'stapler':['stapler'],
 'basket':['wicker basket','fruit basket','basket'],
 'filter':['coffee filter','paper coffee filter','filter paper'],
 'dripper':['coffee dripper','v60','coffee filter holder','pour over'],
 'socket_module':['power outlet','electric socket','wall socket','electrical outlet'],
 'dropper':['pipette','eyedropper','eye dropper','dropper'],
 'jar':['glass jar','mason jar','jar'],
 'tray':['serving tray','plastic tray','cutlery tray','tray'],
 'broom':['hand broom','dust brush','broom'],
 'tape':['packing tape','adhesive tape','tape roll','duct tape'],
 'shelf':['bookshelf','book shelf','shelf'],
 'toy':['rubber duck','toy duck','toy car','toy'],
 'cloth':['cleaning cloth','rag cloth','cloth rag','washcloth','rag'],
 'tomato':['tomato'],
 'tofu':['tofu'],
 'shaker':['cocktail shaker','bartender shaker','shaker'],
 'gpu':['graphics card','graphic card','gpu','geforce'],
 'peeler':['vegetable peeler','potato peeler','peeler'],
 'phone':['smartphone','mobile phone','iphone','phone'],
 'lighter':['lighter','bic lighter'],
 'lipstick':['lipstick'],
 'swab':['cotton swab','cotton bud','q-tip','q tip','cotton stick'],
 'card':['playing card','poker card','card deck','deck of cards'],
 'windmill':['pinwheel','toy windmill','windmill toy','windmill'],
 'rotor':['propeller','pinwheel','fan blade','windmill rotor'],
 'collar':['shaft collar','metal washer','washer','spacer ring'],
 'camera':['dslr camera','digital camera','camera'],
 'photo_target':['photo frame','picture frame','framed picture'],
 'table_tennis':['ping pong table','table tennis table','ping-pong table'],
 'tennis_net':['table tennis net','ping pong net','ping-pong net','tennis net'],
 'paddle':['table tennis racket','ping pong paddle','table tennis paddle','ping-pong racket','ping pong racket'],
 'baton':['relay baton','baton'],
 'scissors':['scissors'],
}
def main():
    matches={k:[] for k in TERMS};scanned=0
    patterns={k:[re.compile(r'(?<![a-z])'+re.escape(t)+r'(?![a-z])',re.I) for t in ts] for k,ts in TERMS.items()}
    for path in sorted((ROOT/'assets/external/objaverse/catalog/metadata').glob('*.gz')):
        data=json.loads(gzip.decompress(path.read_bytes()));scanned+=len(data)
        for uid,a in data.items():
            if a.get('license') not in ('by','cc0','by-sa'):continue
            if a.get('isAgeRestricted'):continue
            title=a.get('name','');tags=' '.join(t['name'] for t in a.get('tags',[]));faces=a.get('faceCount',0);size=(a.get('archives',{}).get('glb') or {}).get('size',0) or 0
            if faces>800000 or size>180000000:continue
            for kind,ps in patterns.items():
                hits=[i for i,p in enumerate(ps) if p.search(title)]
                if not hits:continue
                i=min(hits);score=100-6*i
                if title.lower().strip()==TERMS[kind][i]:score+=30
                if re.search('scan|photogrammetry|realistic|pbr',title+' '+tags,re.I):score+=12
                if re.search('low poly|lowpoly|stylized|cartoon|voxel|minecraft|pixel|origami',title,re.I):score-=35
                if re.search('pack|collection|set of|scene|room|diorama',title,re.I):score-=25
                if 500<=faces<=150000:score+=5
                if a.get('license')=='cc0':score+=8
                score+=min(a.get('likeCount',0),100)/20
                b={'uid':uid,'name':title,'score':score,'license':a.get('license'),'faces':faces,'bytes':size,'url':a.get('viewerUrl'),'author':a.get('user',{}).get('displayName'),'shard':path.stem.split('.')[0],'description':a.get('description','')[:1500],'thumbnails':a.get('thumbnails',{}).get('images',[])}
                heap=matches[kind];heapq.heappush(heap,(score,uid,b))
                if len(heap)>18:heapq.heappop(heap)
    output={k:[x[2] for x in sorted(v,reverse=True)] for k,v in matches.items()}
    (OUT/'candidates.json').write_text(json.dumps(output,ensure_ascii=False,indent=2))
    print('METADATA_OBJECTS',scanned,'MATCHED_KINDS',sum(bool(v) for v in output.values()),'TOTAL_KINDS',len(TERMS))
    for k,v in output.items():print(k,[(x['name'],x['uid'],x['license'],x['faces']) for x in v[:3]])
if __name__=='__main__':main()
