import pygame
from screeninfo import get_monitors

FULLSCREEN = True
RESOLUTION = [0, 0]

if FULLSCREEN:
    """Este loop verifica os ecras todos, para saber a resolução do ecra principal"""
    for m in get_monitors():
        # print(m)
        # if(m.is_primary == True):
        RESOLUTION[0] = m.width
        RESOLUTION[1] = m.height
        # break 

pygame.init()
flags = pygame.DOUBLEBUF | pygame.NOFRAME
SCREEN = pygame.display.set_mode(tuple(RESOLUTION), flags, 8, display=1, vsync=1)
# Aui o display = 0, é que define para por no ecra principal, se puseres 1 mete no secundario e assim sucessivamente
# As flags de DOUBLEBUF e NOFRAME é so para correr um bocadinho mais rapido em fullscreen, nao faz grande diferença mas prontos
pygame.display.set_caption("Main Window")

running = True
while running:
    SCREEN.fill((50, 50, 50))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.update()

pygame.quit()
