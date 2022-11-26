def AddToLog(text, fileName):
    log_file = open(f"C:/debug/{fileName}.txt", "a+")
    log_file.write(text+"\n")
    log_file.close()

    print(text)
