import csv

# Read the CSV file
with open('track_info/tracks/IMS.csv', 'r') as infile:
    reader = csv.reader(infile)
    rows = list(reader)

# Multiply second column by -1
for row in rows:
    if len(row) > 1:
        try:
            row[1] = str(float(row[1]) * -1)
        except ValueError:
            pass  # Skip if not a number

# Write back to CSV
with open('output.csv', 'w', newline='') as outfile:
    writer = csv.writer(outfile)
    writer.writerows(rows)

print("Done! Output saved to output.csv")