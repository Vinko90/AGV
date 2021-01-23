"""
Parses CSV build results to check their result type, and act accordingly.

Call with full path to CSV log file as output.
Expected CSV syntax is <name>;<time>;<result>;<name>
"""

import csv
import sys

def main():
    """
    Checks if all tests in provided CSV passed.
    Exits with an error if any test did not pass
    """
    if len(sys.argv) < 2:
        sys.exit("You must specify the test result file")

    filename = sys.argv[1]
    try:
        with open(filename, 'r') as logfile:
            log_reader = csv.reader(logfile, delimiter=';', quotechar='"')
            results = {}
            total = 0
            for row in log_reader:
                total += 1
                try:
                    results[row[2]] += 1
                except KeyError:
                    results[row[2]] = 1

            print "total: " + str(total)
            for key, count in results.iteritems():
                print key + ": " + str(count)

            if results["PASS"] < total:
                sys.exit("Not all tests passed (" + str(results["PASS"]) + "/" + str(total) + ")")

    except Exception as ex:
        sys.exit("Exception caught: " + ex.message)

if __name__ == '__main__':
    main()
