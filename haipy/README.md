# Hanson AI SDK Python Library

# Memory Manager Example

```python
import datetime
import haipy.memory_manager as mm


mm.init()

# read all performances
performances = mm.Performance.all().to_list()
print(performances)

# find recent performances
performances = mm.Performance.find(mm.Performance.created_at > (datetime.datetime.utcnow() - datetime.timedelta(days=1))).to_list()
print(performances)

# create performance
performance = mm.Performance.find_one(mm.Performance.name == "test2").run()

if not performance:
    performance = mm.Performance(name='test2', timelines = [])
    performance.create()

# update performance
performance.add_timeline([{"type": "speech", "value": "hello"}])
performance.replace()
print(performance)

# person
person = mm.Person.find_one(mm.Person.first_name == "David").run()
if not person:
    person = mm.Person(first_name = 'David', last_name = 'Hanson')
    person.create()
    person.add_attribute('bio', "David Hanson develops robots that are widely regarded as the world’s most human-like in appearance, in a lifelong quest to create true living, caring machines.")
print(person)
```
