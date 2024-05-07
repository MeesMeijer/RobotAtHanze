

# class MyLogger:
#     def __init__(self, output: print):
#         self.out = output
#
#     def error(self, msg, *args):
#         self.out(msg, args)
#
#     def debug(self, msg, *args):
#         self.out(msg, args)
#
#
# logger = MyLogger(print)
# print()
# logger.error("test", "test", "test")
# logger.debug("test debug")
class Coms:

    def write(self, *args, **kwargs):
        print(args, kwargs)

        pass
com = Coms()
print("test1", file=com)