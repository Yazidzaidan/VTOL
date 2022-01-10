import pygame

def init():
    pygame.init()
    win = pygame.display.set_mode((400,400)) #windows pygame

#function to get keypressed
def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans #if press is retur True

def main():
    if getKey("LEFT"):
        print("Left ket pressed")
    if getKey("RIGHT"):
        print("Right ket pressed")

if __name__ == '__main__':
    init()
    while True:
        main()

