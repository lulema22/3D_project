import pygame
import sys
import math
from pygame.locals import *
parA = 2.0
parB = 1.0
dimU = 40
dimV = 20
camDist = 5 
# u_min = -2 * math.pi   # минимальное значение u
# u_max =  2 * math.pi   # максимальное значение u
# v_min = -2             # минимальное значение v
# v_max = 2              # максимальное значение v
# pitch = 0.5            # шаг винта (на сколько растягивается по оси z при увеличении u)




def quatIdent():
    return {'w': 1, 'x':0  ,'y':0, 'z':0}
def quatMultiply(q1, q2):
    return {
        'w': q1['w']*q2['w'] - q1['x']*q2['x']-q1['y']*q2['y'] - q1['z']*q2['z'],
        'x' : q1['w']*q2['x'] + q1['x']*q2['w']+q1['y']*q2['z'] - q1['z']*q2['y'],
        'y': q1['w']*q2['y'] - q1['x']*q2['z']+q1['y']*q2['w'] + q1['z']*q2['x'],
        'z': q1['w']*q2['z'] + q1['x']*q2['y']-q1['y']*q2['x'] + q1['z']*q2['w']
    }
# создаем кватернион по углу и оси
def quatAxis(axis, angle):
    halfAngle = angle /2.0
    l = math.sin(halfAngle)
    return{
        'w' : math.cos(halfAngle),
        'x' : axis[0] *l,
        'y' : axis[1] * l,
        'z' : axis[2]* l
    }
#сопряженный для обратного 
def quatrSopr(q):
    return {'w': q['w'], 'x': -q['x'], 'y': -q['y'], 'z': -q['z']}
#поворот вектора с помощью кватерниона
def quatorReturn(q, vec):
    qp = {'w': 0, 'x': vec[0], 'y': vec[1],'z': vec[2]}
    qv = quatrSopr(q)
    temp = quatMultiply(q, qp)
    res = quatMultiply(temp, qv)
    return (res['x'], res['y'], res['z'])

rotQ = quatIdent()

def tor(R, r ,U, V):
    pts= []
    for i in range(U):
        u = 2*math.pi * i / (U - 1)
        for j in range(V):
            v = 2*math.pi * j/ (V - 1) 
            x = (R + r*math.cos(v))*math.cos(u)
            y = (R + r*math.cos(v))*math.sin(u)
            z = r*math.sin(v)
            pts.append((x, y, z))
    return pts

def helicoid(u, v, pitch=0.5):
    x = v*math.cos(u)
    y = v*math.sin(u)
    z = pitch*u
    return (x, y, z)

def helicoid_surface(uMin, uMax, vMin, vMax, steps_u, steps_v, pitch=0.5):
    pts = []
    for i in range(steps_u):
        u_val = uMin + (uMax - uMin) * i / (steps_u - 1)
        for j in range(steps_v):
            v_val = vMin + (vMax - vMin) * j / (steps_v - 1)
            pts.append(helicoid(u_val, v_val, pitch))
    return pts



def threeTwo(x, y, z, scrW, scrH, camDist, rotQ):
    xRot, yRot, zRot = quatorReturn(rotQ, (x, y, z))
    k = camDist/(camDist - zRot)
    x2d = scrW //2 + xRot* k*100
    y2d = scrH //2 - yRot* k* 100  
    return int(x2d), int(y2d)



def triangl(surface,color, pts, borderColor = None, borderWidth = 0):
    minX = max(0, min(p[0] for p in pts))
    maxX = min(surface.get_width() -1, max(p[0] for p in pts))
    minY = max(0, min(p[1] for p in pts))
    maxY = min(surface.get_height()- 1, max(p[1] for p in pts))
    def ifInTriangl(px, py):
        x1, y1 = pts[0]
        x2, y2 = pts[1]
        x3,y3 = pts[2]

        s1 = (px - x2)*(y1-y2) - (py - y2)*(x1-x2)
        s2 = (px-x1)*(y3-y1) - (py-y1)*(x3-x1)
        s3 = (px-x3)*(y2-y3) - (py-y3)*(x2-x3)
        neg = (s1< 0) or (s2 <0) or (s3< 0)
        pos = (s1> 0) or (s2> 0) or (s3 >0)
        return not(neg and pos)
    
    for py in range(int(minY), int(maxY) + 1):
        for px in range(int(minX), int(maxX) + 1):
            if ifInTriangl(px, py):
                surface.set_at((px, py), color)
    if borderColor and borderWidth >0:
        pygame.draw.line(surface, borderColor, pts[0], pts[1], borderWidth)
        pygame.draw.line(surface, borderColor, pts[0], pts[2], borderWidth)
        pygame.draw.line(surface, borderColor, pts[1], pts[2], borderWidth)

def vecMultiply (v1, v2):
    return (v1[1]*v2[2]-v1[2]*v2[1],
            v1[2]*v2[0]-v1[0]*v2[2],
            v1[0]*v2[1]-v1[1]*v2[0])

def scalMultiply(v1, v2):
    return(v1[0]*v2[0] + v2[1]*v1[1] + v1[2]*v2[2])

def normVec(v):
    l = math.sqrt(v[0]**2 + v[1]**2+ v[2]**2)
    if l == 0:
        return (0, 0, 0)
    return (v[0]/l, v[1]/l, v[2]/l)


def rangeSurface(surface, pts, U, V, scrW, scrH, CameraDist, rotQ):
    lightDir = (1, 1, -1)
    baseColor = (75, 0, 130)
    minLight = 0.2
    for i in range(U):
        for j in range(V):
            iNext = (i+1) % U
            jNext = (j+1)% V
            ptA = pts[i*V + j]
            ptB = pts[iNext*V + j]
            ptC = pts[iNext*V + jNext]
            ptD = pts[i*V +jNext]


            ARot = quatorReturn(rotQ, ptA)
            BRot = quatorReturn(rotQ, ptB)
            CRot = quatorReturn(rotQ, ptC)
            DRot = quatorReturn(rotQ, ptD)

            def vectorTriangl(k1, k2, k3):
                v1 = (k2[0]-k1[0], k2[1]-k1[1], k2[2]-k1[2])
                v2 = (k3[0]-k1[0], k3[1]-k1[1], k3[2]-k1[2])
                normal = normVec(vecMultiply(v1, v2))
                diffuse = max(0, scalMultiply(normal, normVec(lightDir)))
                intens = minLight + (1-minLight)*diffuse
                r = min(255, int(baseColor[0] * intens))
                g = min(255, int(baseColor[1] * intens))
                b = min(255, int(baseColor[2] * intens))
                return (r, g, b)


            A2d = threeTwo(*ptA, scrW, scrH, CameraDist, rotQ)
            B2d = threeTwo(*ptB, scrW, scrH, CameraDist, rotQ)
            C2d = threeTwo(*ptC, scrW, scrH, CameraDist, rotQ)
            D2d = threeTwo(*ptD, scrW, scrH, CameraDist, rotQ)

            color1 = vectorTriangl(ARot, BRot, CRot)
            color2 = vectorTriangl(ARot, DRot, CRot)


            triangl(surface, color1, (A2d, B2d, C2d), (75, 0, 130), 1)
            triangl(surface, color2, (A2d, C2d, D2d), (75, 0, 130), 1)

            



pygame.init()

w, h = 1000, 1000

screen = pygame.display.set_mode((w, h))

clock = pygame.time.Clock()
screen.fill((188, 143, 143))
# pts = helicoid_surface(-2*math.pi, 2*math.pi, -2, 2, dimU, dimV, pitch=0.5)
pts = tor(parA, parB, dimU, dimV)
mouse_down = False
prev_mouse = (0, 0)
auto_rot = False

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                auto_rot = not auto_rot
            elif event.key == pygame.K_UP:
                parA = min(3.0, parA + 0.2)
                # pts = helicoid_surface(-2*math.pi, 2*math.pi, -2, 2, dimU, dimV, pitch=0.5)

                pts = tor(parA, parB, dimU, dimV)
            elif event.key == pygame.K_DOWN:
                parA = max(1.0, parA - 0.2)
                # pts = helicoid_surface(-2*math.pi, 2*math.pi, -2, 2, dimU, dimV, pitch=0.5)

                pts = tor(parA, parB, dimU, dimV)
            elif event.key == pygame.K_RIGHT:
                parB = min(2.0, parB + 0.1)
                # pts = helicoid_surface(-2*math.pi, 2*math.pi, -2, 2, dimU, dimV, pitch=0.5)

                pts = tor(parA, parB, dimU, dimV)
            elif event.key == pygame.K_LEFT:
                parB = max(0.5, parB - 0.1)
                # pts = helicoid_surface(-2*math.pi, 2*math.pi, -2, 2, dimU, dimV, pitch=0.5)

                pts = tor(parA, parB, dimU, dimV)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                mouse_down = True
                auto_rot = False
                prev_mouse = event.pos
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                    mouse_down = False
        elif event.type == pygame.MOUSEMOTION:
            if mouse_down:
                cur = event.pos
                dx = cur[0] - prev_mouse[0]
                dy = cur[1] - prev_mouse[1]
                d_ang_x = math.radians(dx * 0.5)
                d_ang_y = math.radians(dy * 0.5)
                
                q_y = quatAxis((0, 1, 0), d_ang_x)
                    
                q_x = quatAxis((1, 0, 0), d_ang_y)
                rotQ = quatMultiply(q_y, rotQ)
                rotQ = quatMultiply(q_x, rotQ)
                prev_mouse = cur
    if auto_rot:
        q_auto = quatAxis((0, 1, 0), math.radians(0.7))
        rotQ = quatMultiply(q_auto, rotQ)
        pts = helicoid_surface(-2*math.pi, 2*math.pi, -2, 2, dimU, dimV, pitch=0.5)

        # pts = tor(parA, parB, dimU, dimV)
    screen.fill((188, 143, 143))
    rangeSurface(screen, pts, dimU, dimV, w, h, camDist, rotQ)


    origin2d = threeTwo(0, 0, 0, w, h, camDist, rotQ)
    x2d = threeTwo(10, 0, 0, w, h, camDist, rotQ)
    y2d = threeTwo(0, 10, 0, w, h, camDist, rotQ)
    z2d = threeTwo(0, 0, 10, w, h, camDist, rotQ)

    pygame.draw.line(screen, (255, 0, 0), origin2d, x2d, 2)  # ось X – красная
    pygame.draw.line(screen, (0, 255, 0), origin2d, y2d, 2)  # ось Y – зелёная
    pygame.draw.line(screen, (0, 0, 255), origin2d, z2d, 2)  # ось Z – синяя


    
    pygame.display.flip()

    

    clock.tick(60)
#очитска 
pygame.quit()
#завершает программу 
sys.exit()



