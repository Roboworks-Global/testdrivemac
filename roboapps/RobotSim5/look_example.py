objects = look()
for obj in objects:
    shape, color, dist, head, h, w = obj
    print(f"I see a {color} {shape} at distance {dist}")