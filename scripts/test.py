import time
start = time.time()
end = 0
t_thresh = 4.
counter = 0
while end - start < t_thresh:
    print(counter)
    time.sleep(1.)
    end = time.time()
    counter +=1
    print(end - start)
