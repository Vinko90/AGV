import csv
import sys
import os
import Image

HTML_BASE = """
<!DOCTYPE html><html>
<head><b>AGV Nav2d Test-Report</b></head>
<body>
<style type="text/css">
	table{{border-collapse:collapse;}}
	td{{border:1px solid #ddd;padding:8px;}}
	th{{border:1px solid #ddd;padding:8px;}}
</style>
<table>
	<caption><p>Results</p></caption>
<thead>
	<tr><th>LaunchFile</th><th>Execution Time</th><th>Result</th><th>Output Map</th><th>Output Image</th></tr>
</thead>

<tbody>
	{}
</tbody>
</table>

</body></html>
"""

TABLE_ROW = "<tr><td>{}</td><td>{}</td><td><font color=\"{}\">{}</font></td><td>{}</td><td><img src=\"{}\" width=\"300\"></td></tr>"

COLOR_MAP = {
    "PASS": "green",
    "FAIL": "red",
    "PREEMPTED": "red",
    "PENDING/ACTIVE": "orange",
    "UNKNOWN": "#9F000F"
}

def save_png(f, input_dir, output_dir):
    '''
    Pre:    f + ".pgm" is an existing file in input_dir, output_dir exists
    \nPost:   f + ".png" is saved in output_dir
    \nReturns: output filename (without dir)
    '''
    Image.open(input_dir + f + '.pgm').convert('RGB').save(output_dir + f + ".png")
    return f + '.png'

def main():
    if len(sys.argv) < 3:
        print "You must specify a file name to be converted and an output directory"
        sys.exit()

    filename = sys.argv[1]
    inputFolder = os.path.dirname(filename) + "/"
    outputFolder = sys.argv[2] + "/"

    if not os.path.exists(outputFolder):
        os.makedirs(outputFolder)

    try:
        with open(filename, 'r') as logfile:
            logReader = csv.reader(logfile, delimiter=';', quotechar='"')
            table = []
            for row in logReader:
                png_img = save_png(row[3], inputFolder, outputFolder)
                outrow = TABLE_ROW.format(row[0], row[1], COLOR_MAP[row[2]], row[2], row[3], png_img)
                table.append(outrow)

            html = HTML_BASE.format('\n'.join(table))
            htmlLog = open(outputFolder + "log.html", 'w')
            htmlLog.write(html)
            htmlLog.close()
    except Exception:
        print "The file doesn't exist or it is formatted in the wrong way!!"
    

if __name__ == '__main__':
    main()
