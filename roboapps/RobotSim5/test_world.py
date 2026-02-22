# Scattered Object World for v19.2
# Format: Name, Type, Colour, Fixed/Moveable, (w,h,d), (x,y,z), (r,p,y)

# A fixed red pillar
create_object("Pillar1", "cylinder", "Red", "fixed", (0.4, 2.0, 0), (4.0, 2.0, 5.0), (0, 0, 0))

# A moveable yellow crate
create_object("Crate1", "box", "Yellow", "moveable", (0.4, 0.4, 0.4), (4.0, 7.0, 5.0), (0, 0, 45))

# A large blue fixed wall-like box
create_object("Wall1", "box", "Blue", "fixed", (2.0, 0.8, 0.2), (8.0, 3.0, 5.0), (0, 0, 90))

# A small green moveable sphere (ball)
create_object("Ball1", "sphere", "Green", "moveable", (0.5, 0, 0), (7.0, 7.0, 5.0), (0, 0, 0))

# Another moveable cylinder
create_object("Barrel1", "cylinder", "Blue", "moveable", (0.5, 0.8, 0), (2.0, 8.0, 5.0), (0, 0, 0))

# A charger
create_object("charger", "cylinder", "Orange", "fixed", (0.5, 0.8, 0), (0.1, 0.1, 0.1), (0, 0, 0))