import os
import signal
import rccar_control_and_reporting_script

def main_interactive():
  import traceback
  # This gives the error 'OSError: [Errno 1] Operation not permitted' when run by systemd!
  os.setpgrp() # See https://stackoverflow.com/questions/320232/ensuring-subprocesses-are-dead-on-exiting-python-program
  try:
    rccar_control_and_reporting_script.main()
  except KeyboardInterrupt:
    print ("KeyboardInterrupt")
  except: # Print exception before being killed!
    traceback.print_exc()
  finally:
    os.killpg(0, signal.SIGTERM)

if __name__ == '__main__':
  main_interactive()