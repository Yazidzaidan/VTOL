import pandas as pd

# method 1 : list of lists
#data_1 = [ ['Yazid', 21, 'Apple'], ['Azhara', 21, 'Mango'], ['Zaidan', 22, 'Cherry'], ['Distyani', 25, 'Durian'] ]

#df_1 = pd.DataFrame(data_1, columns=['Name', 'Age', 'Fav Fruit'])
#print(df_1)


# method 2 : dict of lists
data_2 = {'Name':['Adi','Budi','Candra','Dewi'],
          'Age':[10,20,35,22],
          'Fav Fruit':['Apple','Orange','Cherry','Avocado']}

df_2 = pd.DataFrame(data_2) # tidak perlu kasih column name karena sudah ambil dari key yang ada di dalam dict
print(df_2)

df_2.to_excel('test_2.xlsx', index=False)