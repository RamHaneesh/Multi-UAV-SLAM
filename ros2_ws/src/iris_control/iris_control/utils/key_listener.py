import sys, termios, tty, select

# save terminal settings ONCE
settings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())

    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = ''

    if rlist:
        key = sys.stdin.read(1)

        # Ctrl+C handling
        if key == '\x03':
            raise KeyboardInterrupt

        # handle arrow keys
        if key == '\x1b':
            key += sys.stdin.read(2)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key

def main():
    print("Press keys (Ctrl+C to exit)...")

    try:
        while True:
            key = get_key()

            if key:
                print(f"Raw key: {repr(key)}")

                if key == '\x1b[A':
                    print("UP ARROW")
                elif key == '\x1b[B':
                    print("DOWN ARROW")
                elif key == '\x1b[C':
                    print("RIGHT ARROW")
                elif key == '\x1b[D':
                    print("LEFT ARROW")
                else:
                    print(f"Other key: {key}")

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        # 🔥 THIS IS THE IMPORTANT PART
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Terminal restored.")

if __name__ == "__main__":
    main()