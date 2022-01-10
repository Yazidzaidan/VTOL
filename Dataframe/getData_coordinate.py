#contoh penarikan data dari excel

import pandas as pd
#from openpyxl import load_workbook


#data_coba = pd.read_excel('E:\YAZID\KOMPOR UPI\KRTI 2021\Riset\Origin3\coordinate.xlsx')

df = pd.read_excel('coordinate and area.xlsx')
#df.info()
#print(df)
df_sort = df.tail(14)
#df_sort.info()
print(df_sort)

#print(df_sort.iat[-2,0])

coorAx = df_sort['CoordX'].values[0]
coorAy = df_sort['CoordY'].values[0]
coorBx = df_sort['CoordX'].values[1]
coorBy = df_sort['CoordY'].values[1]
coorCx = df_sort['CoordX'].values[2]
coorCy = df_sort['CoordY'].values[2]
coorObsx = df_sort['CoordX'].values[3]
coorObsy = df_sort['CoordY'].values[3]
area1Ax = df_sort['CoordX'].values[4]
area1Ay = df_sort['CoordY'].values[4]
area1Bx = df_sort['CoordX'].values[5]
area1By = df_sort['CoordY'].values[5]
area2Ax = df_sort['CoordX'].values[6]
area2Ay = df_sort['CoordY'].values[6]
area2Bx = df_sort['CoordX'].values[7]
area2By = df_sort['CoordY'].values[7]
area3Ax = df_sort['CoordX'].values[8]
area3Ay = df_sort['CoordY'].values[8]
area3Bx = df_sort['CoordX'].values[9]
area3By = df_sort['CoordY'].values[9]
area4Ax = df_sort['CoordX'].values[10]
area4Ay = df_sort['CoordY'].values[10]
area4Bx = df_sort['CoordX'].values[11]
area4By = df_sort['CoordY'].values[11]
area5Ax = df_sort['CoordX'].values[12]
area5Ay = df_sort['CoordY'].values[12]
area5Bx = df_sort['CoordX'].values[13]
area5By = df_sort['CoordY'].values[13]
print("Gd A : ", (coorAx,coorAy))
print("Gd B : ", (coorBx,coorBy))
print("Gd C : ", (coorCx,coorCy))
print("Obstacle : ", (coorObsx,coorObsy))
print("Area 1A : ", (area1Ax,area1Ay))
print("Area 1B : ", (area1Bx,area1By))
print("Area 2A : ", (area2Ax,area2Ay))
print("Area 2B : ", (area2Bx,area2By))
print("Area 3A : ", (area3Ax,area3Ay))
print("Area 3B : ", (area3Bx,area3By))
print("Area 4A : ", (area4Ax,area4Ay))
print("Area 4B : ", (area4Bx,area4By))
print("Area 5A : ", (area5Ax,area5Ay))
print("Area 5B : ", (area5Bx,area5By))


#df = pd.read_excel(r'E:\YAZID\KOMPOR UPI\KRTI 2021\Riset\Origin3\coordinate.xlsx')
#df = pd.read_excel(open('coordinate.xlsx', , engine='openpyxl'),
#              sheet_name='Sheet1')
#print (df)

#path = "coordinate.xlsx"
#book = load_workbook(path)
#writer = pd.ExcelWriter("coordinate.xlsx", engine='openpyxl')

#writer.book = book
#writer.sheets = {ws.title: ws for ws in book.worksheets}
#df_gd.to_excel(writer, startrow=writer.sheets['Sheet1'].max_row, index = False,header= False)

#writer.save()
#print(book)