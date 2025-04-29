import pygame
import time

# Initialize pygame
pygame.init()

# Create an invisible window (you need a display for event handling)
screen = pygame.display.set_mode((1, 1))

# Main loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        # Check for keydown events
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                print('Key Up')
            elif event.key == pygame.K_DOWN:
                print('Key Down')
            else:
                print('Invalid Key')

    # Add a delay (optional)
    time.sleep(0.5)
