from django.urls import path
from traveller import views

urlpatterns = [path("getroute/", views.ObtainBestRoute.as_view(), name="route")]
