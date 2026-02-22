# Create the perimeter walls (optional, as the engine provides basic boundaries)
create_object("Wall_North", "box", "White", "static", [10, 0.2, 1.0], [5.0, 9.9, 0], [0, 0, 0])
create_object("Wall_South", "box", "White", "static", [10, 0.2, 1.0], [5.0, 0.1, 0], [0, 0, 0])

# Central Pillars (Static Obstacles)
create_object("Pillar_1", "cylinder", "Blue", "static", [0.8, 0.8, 2.0], [3.0, 3.0, 0], [0, 0, 0])
create_object("Pillar_2", "cylinder", "Blue", "static", [0.8, 0.8, 2.0], [7.0, 7.0, 0], [0, 0, 0])

# Charging Station (Provides energy when robot is within 0.6 units)
create_object("Main_Charger", "box", "Yellow", "static", [1.0, 1.0, 0.1], [1.0, 9.0, 0], [0, 0, 0])

# Moveable Cargo (Can be picked up with the 'attach()' command)
create_object("Crate_Small", "box", "Orange", "moveable", [0.4, 0.4, 0.4], [5.0, 5.0, 0], [0, 0, 45])
create_object("Crate_Large", "box", "Orange", "moveable", [0.6, 0.6, 0.6], [8.0, 2.0, 0], [0, 0, 0])
create_object("Barrel", "cylinder", "Green", "moveable", [0.4, 0.4, 0.6], [2.0, 6.0, 0], [0, 0, 0])

# Decorative Sphere (To test LiDAR curvature)
create_object("Orb", "sphere", "White", "static", [0.5, 0.5, 0.5], [8.5, 8.5, 0], [0, 0, 0])