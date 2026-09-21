"""Small mouse/keyboard terrain panel drawn inside the existing OpenGL window."""
import mujoco

class TerrainMenu:
    fields = [
        ("preset", "Terrain", ["flat", "curved", "bumpy"]),
        ("length", "Arc length (m)", (2.,30.,1.)),
        ("angle", "Arc angle (deg)", (5.,90.,5.)),
        ("width", "Plate width (m)", (1.5,10.,.5)),
        ("bump_diameter", "Bump diameter (m)", (.02,.5,.01)),
        ("bump_height", "Bump height (m)", (.001,.05,.001)),
        ("tilt", "Tilt: wall 0 / underside 90", (0.,90.,5.)),
        ("inward", "Curvature", [False,True]),
        ("bump_spacing", "Bump longitudinal spacing (m)", (.15,5.,.05)),
        ("bump_track_spacing", "Bump lateral spacing (m)", (.1,9.9,.05)),
    ]
    def __init__(self, **values):
        self.values={"bump_spacing": .8, "bump_track_spacing": .737, **values}
        self.visible=False
        self.selected=0
        self.pending=False
        self.message="Edit, then Apply. Robot resets and stays paused."

    def adjust(self, step):
        name,_,choices=self.fields[self.selected]
        if isinstance(choices,list):
            self.values[name]=choices[(choices.index(self.values[name])+step)%len(choices)]
        else:
            lo,hi,delta=choices
            self.values[name]=round(min(hi,max(lo,self.values[name]+step*delta)),6)

    def layout(self,w,h):
        width=min(620,w-24)
        row=min(44,max(22,(h-160)//(len(self.fields)+2)))
        x=(w-width)//2
        top=h-60
        return x,width,row,top

    def click(self,x,y,w,h):
        left,width,row,top=self.layout(w,h)
        if not left<=x<=left+width:return
        index=int((top-55-y)//row)
        if 0<=index<len(self.fields):
            self.selected=index
            if x>left+width-70:self.adjust(1)
            elif x>left+width-140:self.adjust(-1)
        elif index==len(self.fields):self.pending=True

    def draw(self,w,h,context):
        left,width,row,top=self.layout(w,h)
        def box(x,y,bw,bh,color):
            rect=mujoco.MjrRect(int(x),int(y),int(bw),int(bh))
            mujoco.mjr_rectangle(rect,*color)
            return rect
        def text(rect,label):
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               rect,label,"",context)
        rows=len(self.fields)+2
        box(left-8,top-55-rows*row,width+16,rows*row+100,(.10,.14,.19,1))
        text(mujoco.MjrRect(left,top,width,35),"TERRAIN SETTINGS  |  M / Esc: close")
        text(mujoco.MjrRect(left,top-32,width,30),"Mouse +/- or arrows; Enter = Apply")
        for i,(name,label,_) in enumerate(self.fields):
            bottom=top-55-(i+1)*row
            active=i==self.selected
            rect=box(left,bottom,width,row-3,(.22,.31,.40,1) if active else (.15,.20,.27,1))
            value=self.values[name]
            if name=="inward":value="concave" if value else "convex"
            elif isinstance(value,float):value=f"{value:g}"
            disabled=self.values["preset"]=="flat" and i>0
            text(rect,f"{label}: {value}"+(" (unused for flat)" if disabled else ""))
            text(mujoco.MjrRect(left+width-132,int(bottom),60,row-3),"[-]")
            text(mujoco.MjrRect(left+width-62,int(bottom),60,row-3),"[+]")
        rect=box(left,top-55-(len(self.fields)+1)*row,width,row-3,(.17,.40,.35,1))
        text(rect,"APPLY & RESET ROBOT")
        text(mujoco.MjrRect(left,int(top-55-(len(self.fields)+2)*row),width,row),self.message[:70])
