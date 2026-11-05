"""Score a simulated mission: inventory output (SQLite queue or CSV) vs ground_truth.json.

Metrics (the team's regression metric, see docs/simulation.md):
  read recall        items read / items on the shelves
  false reads        decoded item codes that are not on the shelves
  correct-slot rate  items attributed to the right slot / items read
"""
